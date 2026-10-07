/* [AI] R38: encoder_sign 持久化实现——见 app_sign_store.h 头注。
 * 仅依赖 HAL Flash API；无动态内存、无浮点、ISR 内禁调（save 在任务态调用）。 */
#include "app_sign_store.h"

#include "stm32f4xx_hal.h"

#include <string.h>

typedef struct {
    uint32_t magic;
    uint32_t seq;
    int8_t   signs[4];
    uint32_t crc;      /* 覆盖前 12 字节 */
} sign_record_t;

/* [AI] FNV-1a 32：极性记录用不到强校验，抗位翻转够用且无查表开销 */
static uint32_t sign_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t hash = 0x811C9DC5UL;
    for (uint32_t i = 0; i < len; ++i) {
        hash ^= data[i];
        hash *= 0x01000193UL;
    }
    return hash;
}

static inline uintptr_t slot_addr(uint32_t index)
{
    return (uintptr_t)(SIGN_STORE_BASE + (uintptr_t)index * 16U);
}

static uint8_t slot_empty(uintptr_t addr)
{
    /* [AI] 擦除态 = 全 0xFF（16B 槽首 doubleword 非 FF 即已用） */
    return (*(volatile uint32_t *)addr == 0xFFFFFFFFUL);
}

uint32_t sign_store_prepare(void)
{
    for (uint32_t i = 0; i < SIGN_STORE_SLOT_N; ++i) {
        if (slot_empty(slot_addr(i))) {
            return 0U;   /* 还有空槽，无需擦除 */
        }
    }
    FLASH_EraseInitTypeDef erase;
    uint32_t error = 0U;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = SIGN_STORE_SECTOR;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /* 2.7-3.6V 供电 */
    if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK || error != 0xFFFFFFFFUL) {
        return 1U;
    }
    return 0U;
}

uint8_t sign_store_load(int8_t signs_out[4])
{
    uint8_t found = 0U;
    uint32_t best_seq = 0U;
    for (uint32_t i = 0; i < SIGN_STORE_SLOT_N; ++i) {
        const sign_record_t *rec = (const sign_record_t *)slot_addr(i);
        if (rec->magic != SIGN_STORE_MAGIC) {
            continue;   /* 擦除态或坏槽 */
        }
        uint32_t crc = sign_crc32((const uint8_t *)rec, 12U);
        if (crc != rec->crc) {
            continue;   /* 位翻转/半写记录：跳过 */
        }
        if (rec->signs[0] == 0 || rec->signs[1] == 0 ||
            rec->signs[2] == 0 || rec->signs[3] == 0) {
            continue;
        }
        if (!found || rec->seq > best_seq) {
            best_seq = rec->seq;
            memcpy(signs_out, rec->signs, 4U);
            found = 1U;
        }
    }
    return found;
}

uint32_t sign_store_save(const int8_t signs[4])
{
    if (signs == NULL || signs[0] == 0 || signs[1] == 0 ||
        signs[2] == 0 || signs[3] == 0) {
        return 2U;   /* 未学满不落盘 */
    }
    /* [AI] 取当前最高 seq+1；全空从 1 起 */
    uint32_t last_seq = 0U;
    int32_t free_slot = -1;
    for (uint32_t i = 0; i < SIGN_STORE_SLOT_N; ++i) {
        uintptr_t addr = slot_addr(i);
        if (slot_empty(addr)) {
            free_slot = (int32_t)i;
            break;
        }
        const sign_record_t *rec = (const sign_record_t *)addr;
        if (rec->magic == SIGN_STORE_MAGIC && rec->crc ==
            sign_crc32((const uint8_t *)rec, 12U) && rec->seq > last_seq) {
            last_seq = rec->seq;
        }
    }
    if (free_slot < 0) {
        return 3U;   /* 满了：留给下次开机 prepare() 擦除 */
    }

    sign_record_t rec;
    rec.magic = SIGN_STORE_MAGIC;
    rec.seq = last_seq + 1U;
    memcpy(rec.signs, signs, 4U);
    rec.crc = sign_crc32((const uint8_t *)&rec, 12U);

    /* [AI] R38 联测修正（顺序关键）：SR 错误标志（PGSERR/PGPERR 等）粘滞——
     * 软复位不清除、ST-Link 下载流程会遗留。HAL_FLASH_Unlock 内部先跑
     * WaitForLastOperation，见到这些标志会直接 HAL_ERROR 且不解锁 CR——
     * 所以清标志必须放在 Unlock 之前，否则永远锁死（实测 car_init 隔离
     * 实验定位）。 */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    HAL_FLASH_Unlock();
    /* [AI] R38 联测：DOUBLEWORD 编程实测 PGAERR|PGSERR（原因待查，疑与
     * CPU 同 bank 取指执行有关），改用 4 次 WORD 编程（x32 并行，3.3V 合法）。 */
    for (uint32_t w = 0; w < 4U; ++w) {
        uint32_t word;
        memcpy(&word, ((const uint8_t *)&rec) + w * 4U, 4U);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              slot_addr((uint32_t)free_slot) + w * 4U,
                              word) != HAL_OK) {
            HAL_FLASH_Lock();
            return 4U;
        }
    }
    HAL_FLASH_Lock();

    /* [AI] 回读校验：Flash 编程失败静默会埋下下次开机预载坏极性的雷 */
    const sign_record_t *verify = (const sign_record_t *)slot_addr((uint32_t)free_slot);
    if (verify->magic != rec.magic || verify->seq != rec.seq ||
        memcmp(verify->signs, rec.signs, 4U) != 0 || verify->crc != rec.crc) {
        return 5U;
    }
    return 0U;
}
