#include "eeprom.h"

/*
 * 该文件包含对EEPROM的操作函数，用于存储和读取电感采集数据的最大值和最小值。
 * 这些值用于在不同场地和不同信号发生器规格下，确保电感识别的范围一致。
 * 通过将当前采集的数值进行归一化处理（value*100）/(max-min)，得到一个数，用于PID调节。
 */

/*
 * 用于初始化读取0x00地址的数据
 * 初始化时读取两个字节，用于判断是否已有数据
 * 如果EEPROM的第0页未初始化，这两个字节可能包含无效数据
 */
uint8 init_buff[2] = {0};

/**
 * @brief 初始化EEPROM模块
 * 该函数负责初始化EEPROM模块，主要调用iap_init()函数进行内部初始化。
 * 初始化后，读取EEPROM中的最大值和最小值，并检查是否已经初始化。
 * 如果未初始化，则擦除该页并进行重新初始化。
 */
void eeprom_init(void)
{
    iap_init();                      // 调用内部iap初始化函数
    eeprom_read(0x00, init_buff, 2); // 读取两个字节，分别对应状态标志

    // 如果状态标志为0，则表示未初始化，擦除并重新写入
    if (!(init_buff[0] == 0x00 && init_buff[1] == 0x00))
    {
        iap_erase_page(0); // 擦除EEPROM第0页
    }
}

/**
 * @brief 从EEPROM指定地址读取数据
 * 从EEPROM指定的地址读取数据，并存储到数据缓冲区中。
 * @param addr EEPROM中的地址
 * @param buf 数据缓冲区指针，指向用于存储读取数据的缓冲区
 * @param len 要读取的数据长度（以字节为单位）
 * @return 无
 * tip:内部调用，无需使用
 */
void eeprom_read(uint32 addr, uint8 *buf, uint16 len)
{
    iap_read_buff(addr, buf, len); // 调用iap_read_buff函数读取数据
}

/**
 * @brief 将最大值和最小值写入EEPROM
 * 将电感采集数据的最大值和最小值写入EEPROM，用于后续的数据处理。
 * @param addr EEPROM中的起始地址
 * @param max 最大值（16位整数）
 * @param min 最小值（16位整数）
 * @return 无
 */
void eeprom_write_value(uint32 addr, uint16 max, uint16 min)
{
    /*
     * 将电感采集的最大值和最小值各拆分为两个8位字节，并写入到EEPROM中
     * max的低8位：max & 0xFF
     * max的高8位：(max >> 8) & 0xFF
     * min的低8位：min & 0xFF
     * min的高8位：(min >> 8) & 0xFF
     */
    iap_write_byte(addr, (max >> 8) & 0xFF);     // 写入max的高8位
    iap_write_byte(addr + 1, max & 0xFF);        // 写入max的低8位
    iap_write_byte(addr + 2, (min >> 8) & 0xFF); // 写入min的高8位
    iap_write_byte(addr + 3, min & 0xFF);        // 写入min的低8位
}

/**
 * @brief 从EEPROM读取最大值和最小值
 * 从EEPROM指定的地址读取最大值和最小值，并存储到数据缓冲区中。
 * @param addr EEPROM中的起始地址
 * @param buff 数据缓冲区指针，指向用于存储最大值和最小值的缓冲区
 *             buff[0]将存储最大值
 *             buff[1]将存储最小值
 * @return 无
 */
void eeprom_read_value(uint32 addr, uint16 *buff)
{
    uint8 buf[4];
    iap_read_buff(addr, buf, 4); // 调用iap_read_buff函数读取数据

    // 合并两个字节为16位整数
    buff[0] = (buf[0] << 8) | buf[1]; // 合并为最大值
    buff[1] = (buf[2] << 8) | buf[3]; // 合并为最小值
}
