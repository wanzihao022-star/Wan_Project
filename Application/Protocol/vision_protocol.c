/**
 * @file    vision_protocol.c
 * @brief   高复用视觉协议 V4.0 实现
 */

#include "vision_protocol.h"

/*============================ 全局变量定义 ============================*/

volatile VisionPacket_t g_vision = {0};                 ///< 全局数据包（零初始化）
uint8_t g_vision_rx_buf[VISION_RX_BUF_SIZE] = {0};      ///< DMA接收缓冲区

static UART_HandleTypeDef *s_huart = NULL;              ///< 串口句柄（内部保存）
static const FrameConfig_t *s_current_cfg = NULL;       ///< 当前使用的配置

/*============================ 预定义配置表 ============================*/

/**
 * @brief 第二次积分赛标准配置
 * @details
 *   帧结构（13字节）：
 *   [AA] [55] [CMD] [dx_L] [dx_H] [dy_L] [dy_H] [type] [phase_L] [phase_H] [rpm_L] [rpm_H] [chk]
 *
 *   字段说明：
 *   - dx/dy:   激光点相对目标中心偏差，像素，有符号16位，原值传输（scale=1.0）
 *   - valid:   0=无效，1=有效，2=激光丢失，3=有效且带相位（用于任务5）
 *   - type:    图形类型（1=圆，3=三角，4=方，5=五边形等）
 *   - phase:   旋转相位，0-65535对应0-360°，无符号16位，scale=65535/360
 *   - rpm:     转速，有符号16位，scale=100（表示百分之一rpm，如750=7.5rpm）
 */
const FrameConfig_t g_config_QD4310 = {
    .name = "QD4310",
    .sof = {0xAA, 0x55},
    .field_num = 5,   //5个有效字段
    .fields = {
        {FIELD_VALID,     1, 0, 1.0f},                          ///< valid: uint8，原值
        {FIELD_DX,        2, 1, 1.0f},                          ///< dx: int16，原值
        {FIELD_DY,        2, 1, 1.0f},                          ///< dy: int16，原值
        {FIELD_PHASE,     2, 0, 65535.0f / 360.0f},             ///< phase: uint16，360°映射到65535< 旋转相位（0-65535）
        {FIELD_RPM,       2, 1, 100.0f},                        ///< rpm: int16，百分之一精度
    }
};

/**
 * @brief 第一次积分赛配置（示例）
 * @details 仅传输dx和valid，用于简单循迹小车
 */
const FrameConfig_t g_config_linefollow = {
    .name = "LineFollow",
    .sof = {0xAA, 0x55},
    .field_num = 2,
    .fields = {
        {FIELD_VALID,     1, 0, 1.0f},      ///< 是否看到线
        {FIELD_DX,        2, 1, 1.0f},      ///< 只关心横向偏差
    }
};

/**
 * @brief 多目标识别配置（示例）
 * @details 传输目标数量+多个坐标（演示扩展用法）
 */
const FrameConfig_t g_config_multitarget = {
    .name = "MultiTarget",
    .sof = {0xAA, 0x55},
    .field_num = 5,
    .fields = {
        {FIELD_VALID,     1, 0, 1.0f},      ///< 目标数量
        {     FIELD_TYPE,      1, 0, 1.0f},      ///< 当前目标索引
        {FIELD_DX,        2, 1, 1.0f},      ///< 当前目标X
        {FIELD_DY,        2, 1, 1.0f},      ///< 当前目标Y
        {FIELD_CONFIDENCE,1, 0, 1.0f},      ///< 识别置信度
        {FIELD_CUSTOM0,   1, 0, 1.0f},              ///< 当前目标索引（custom[0]
    }
};

/*============================ 内部工具函数 ============================*/

/**
 * @brief 计算帧总长度（根据配置表自动计算）
 * @param cfg 配置表指针
 * @return 帧字节数 = 帧头(2) + 所有字段长度和 + 校验和(1)
 */
static uint8_t CalcFrameLen(const FrameConfig_t *cfg)
{
    if (!cfg) return 0;

    uint8_t len = 2;  // 帧头 SOF0 + SOF1
    for (uint8_t i = 0; i < cfg->field_num; i++) {
        len += cfg->fields[i].size;
    }
    len += 1;  // 校验和
    return len;
}

/**
 * @brief 辅助函数：计算累加校验和
 * @param data 数据指针
 * @param len  计算长度（不含校验和本身）
 * @return 校验和（低8位）
 */
static uint8_t CalcChecksum(uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/**
 * @brief 将VisionPacket_t中的字段写入发送缓冲区（打包）
 * @param cfg   配置表
 * @param pkt   数据源
 * @param out   输出缓冲区（至少VISION_MAX_FRAME_LEN字节）
 * @return 实际写入的字节数（帧长度）
 * @note  小端模式写入，自动应用scale缩放
 * ****************这里是打包的函数，可用于回环测试，也可以以后发给相机做测验**************
 */
static uint8_t PackFrame(const FrameConfig_t *cfg, const VisionPacket_t *pkt, uint8_t *out)
{
    uint8_t pos = 0;

    // 写入帧头
    out[pos++] = cfg->sof[0];
    out[pos++] = cfg->sof[1];

    // 按配置表顺序写入各字段
    for (uint8_t i = 0; i < cfg->field_num; i++) {
        const FieldDef_t *f = &cfg->fields[i];
        int32_t val = 0;

        // 根据字段类型从结构体取值并缩放
        switch (f->type) {
            case FIELD_VALID:     val = (int32_t)(pkt->valid / f->scale); break;
            case FIELD_DX:        val = (int32_t)(pkt->dx / f->scale); break;
            case FIELD_DY:        val = (int32_t)(pkt->dy / f->scale); break;
            case FIELD_TYPE:      val = (int32_t)(pkt->type / f->scale); break;
            case FIELD_PHASE:     val = (int32_t)(pkt->phase / f->scale); break;
            case FIELD_RPM:       val = (int32_t)(pkt->rpm / f->scale); break;
            case FIELD_CONFIDENCE:val = (int32_t)(pkt->confidence / f->scale); break;
            case FIELD_TIMESTAMP: val = (int32_t)(pkt->timestamp / f->scale); break;
            case FIELD_CUSTOM0:   val = (int32_t)(pkt->custom[0] / f->scale); break;
            case FIELD_CUSTOM1:   val = (int32_t)(pkt->custom[1] / f->scale); break;
            case FIELD_CUSTOM2:   val = (int32_t)(pkt->custom[2] / f->scale); break;
            case FIELD_CUSTOM3:   val = (int32_t)(pkt->custom[3] / f->scale); break;
            default:              val = 0; break;
        }

        // 小端写入指定字节数
        for (uint8_t b = 0; b < f->size; b++) {
            out[pos++] = (uint8_t)((val >> (b * 8)) & 0xFF);
        }
    }

    // 计算并写入校验和（校验除校验和外的所有字节）
    out[pos] = CalcChecksum(out, pos);
    pos++;

    return pos;
}

/**
 * @brief 从接收缓冲区解析帧（解包）
 * @param cfg   配置表（决定如何解析）
 * @param data  接收数据（可能包含多个帧或脏数据）
 * @param len   数据长度
 * @param out   输出结构体（解析成功时填充）
 * @return true=找到有效帧且校验通过
 * @note  自动在data中查找帧头，支持非对齐数据
 */
static bool UnpackFrame(const FrameConfig_t *cfg, uint8_t *data, uint16_t len, VisionPacket_t *out)
{
    if (!cfg || !data || !out) return false;

    uint8_t frame_len = CalcFrameLen(cfg);
    if (len < frame_len) return false;  // 数据不够一帧

    // 在缓冲区中查找帧头
    for (uint16_t i = 0; i <= len - frame_len; i++) {
        // 匹配帧头
        if (data[i] != cfg->sof[0] || data[i+1] != cfg->sof[1]) {
            continue;
        }

        // 校验检查（除最后一个字节外累加）
        uint8_t calc_chk = CalcChecksum(&data[i], frame_len - 1);
        if (calc_chk != data[i + frame_len - 1]) {
            continue;  // 校验失败，继续找下一帧
        }

        // 校验通过，解析字段
        memset(out, 0, sizeof(VisionPacket_t));  // 清零（未配置字段默认为0）
        uint8_t pos = i + 2;  // 跳过帧头

        for (uint8_t f = 0; f < cfg->field_num; f++) {
            const FieldDef_t *fd = &cfg->fields[f];

            // 小端读取
            int32_t raw_val = 0;
            for (uint8_t b = 0; b < fd->size; b++) {
                raw_val |= (int32_t)(data[pos + b]) << (b * 8);
            }

            // 符号扩展（对于有符号且小于4字节的字段）
            if (fd->is_signed && fd->size < 4) {
                int32_t sign_bit = 1 << (fd->size * 8 - 1);
                if (raw_val & sign_bit) {
                    raw_val |= ~((1 << (fd->size * 8)) - 1);  // 高位补1
                }
            }

            // 应用缩放并写入结构体
            float real_val = (float)raw_val * fd->scale;
            switch (fd->type) {
                case FIELD_VALID:     out->valid = (uint8_t)real_val; break;
                case FIELD_DX:        out->dx = (int16_t)real_val; break;
                case FIELD_DY:        out->dy = (int16_t)real_val; break;
                case FIELD_TYPE:      out->type = (uint8_t)real_val; break;
                case FIELD_PHASE:     out->phase = (uint16_t)real_val; break;
                case FIELD_RPM:       out->rpm = (int16_t)real_val; break;
                case FIELD_CONFIDENCE:out->confidence = (uint8_t)real_val; break;
                case FIELD_TIMESTAMP: out->timestamp = (uint32_t)real_val; break;
                case FIELD_CUSTOM0:   out->custom[0] = (int16_t)real_val; break;
                case FIELD_CUSTOM1:   out->custom[1] = (int16_t)real_val; break;
                case FIELD_CUSTOM2:   out->custom[2] = (int16_t)real_val; break;
                case FIELD_CUSTOM3:   out->custom[3] = (int16_t)real_val; break;
                default: break;
            }

            pos += fd->size;
        }

        out->recv_time = HAL_GetTick();  // 记录接收时间戳
        out->data_valid = 1;             // 标记解析成功
        return true;
    }

    return false;  // 未找到有效帧
}

/*============================ 接口函数实现 ============================*/
//初始化，必须要有，例如Vision_Init(&huart6);
void Vision_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    // 肯定默认选择QD4310（第二次积分赛）配置
    s_current_cfg = &g_config_QD4310;

    // 清零全局数据
    memset((void*)&g_vision, 0, sizeof(g_vision));
    memset(g_vision_rx_buf, 0, VISION_RX_BUF_SIZE);

    // 启动DMA接收（空闲中断模式）
    if (huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_vision_rx_buf, VISION_RX_BUF_SIZE);
    }
}
//选择协议配置
bool Vision_SelectConfig(const char *name)
{
    const FrameConfig_t *new_cfg = NULL;

    // 根据名称选择配置（在此处添加新配置）
    if (strcmp(name, "QD4310") == 0) {
        new_cfg = &g_config_QD4310;
    } else if (strcmp(name, "LineFollow") == 0) {
        new_cfg = &g_config_linefollow;
    } else if (strcmp(name, "MultiTarget") == 0) {
        new_cfg = &g_config_multitarget;
    } else {
        return false;  // 未找到配置
    }

    // 切换配置
    s_current_cfg = new_cfg;

    // 重置接收（清空缓冲区，防止新旧配置帧格式冲突）
    memset(g_vision_rx_buf, 0, VISION_RX_BUF_SIZE);
    if (s_huart) {
        HAL_UART_AbortReceive(s_huart);
        HAL_UARTEx_ReceiveToIdle_DMA(s_huart, g_vision_rx_buf, VISION_RX_BUF_SIZE);
    }

    return true;
}
//UART DMA接收完成回调（必须放入HAL中断回调）
void Vision_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != s_huart || !s_current_cfg) return;

    VisionPacket_t temp;

    // 尝试解析
    if (UnpackFrame(s_current_cfg, g_vision_rx_buf, Size, &temp)) {
        // 原子拷贝到全局变量（关中断保护，防止读取时被中断修改）
        __disable_irq();
        memcpy((void*)&g_vision, &temp, sizeof(VisionPacket_t));
        __enable_irq();
    }

    // 重新启动DMA接收（不清零缓冲区，复用内存）
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, g_vision_rx_buf, VISION_RX_BUF_SIZE);
}
//发送命令到相机（C板->MaixCam）
bool Vision_SendCommand(uint8_t cmd, int16_t param)
{
    if (!s_huart) return false;

    // 命令帧格式：AA 55 CMD_L CMD_H PARAM_L PARAM_H 00 00 CHK（8字节）
    uint8_t tx_buf[9];
    tx_buf[0] = 0xAA;
    tx_buf[1] = 0x55;
    tx_buf[2] = cmd & 0xFF;
    tx_buf[3] = 0;  // CMD高字节（保留）
    tx_buf[4] = param & 0xFF;
    tx_buf[5] = (param >> 8) & 0xFF;
    tx_buf[6] = 0;  // 预留
    tx_buf[7] = 0;  // 预留
    tx_buf[8] = CalcChecksum(tx_buf, 8);

    return (HAL_UART_Transmit(s_huart, tx_buf, 9, 100) == HAL_OK);
}
//获取当前使用的配置指针（调试用）
const FrameConfig_t* Vision_GetCurrentConfig(void)
{
    return s_current_cfg;
}
//计算当前配置的帧长度（调试用）
uint8_t Vision_GetFrameLen(void)
{
    return s_current_cfg ? CalcFrameLen(s_current_cfg) : 0;
}
//手动触发数据解析（测试用）
bool Vision_ParseManual(uint8_t *raw_data, uint16_t len, VisionPacket_t *out)
{
    if (!s_current_cfg) return false;
    return UnpackFrame(s_current_cfg, raw_data, len, out);
}

// 示例：发送完整的视觉数据帧（用于回环测试或调试）
bool Vision_SendPacket(const VisionPacket_t* pkt)
{
    if (!s_huart || !s_current_cfg) return false;

    uint8_t tx_buf[VISION_MAX_FRAME_LEN];
    uint8_t len = PackFrame(s_current_cfg, pkt, tx_buf);

    return (HAL_UART_Transmit(s_huart, tx_buf, len, 100) == HAL_OK);
}
