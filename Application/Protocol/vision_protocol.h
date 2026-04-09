/**
 * @file    vision_protocol.h
 * @brief   高复用视觉通信协议 V4.0 - 配置驱动型串口协议栈
 * @author  万梓豪
 * @date    2026-04-01
 * @version 4.0
 *
 * @description
 * 本协议栈采用"配置表驱动"架构，通过修改配置表即可适配不同比赛/应用场景，
 * 无需修改解析代码。支持固定帧长、校验和、小端模式、自动缩放。
 *
 * @usage
 * 1. 在 main.c 中调用 Vision_Init() 初始化
 * 2. 调用 Vision_SelectConfig() 选择当前比赛配置
 * 3. 在 USART DMA 中断中调用 Vision_RxCallback()
 * 4. 直接读取全局变量 g_vision 获取最新数据
 */

#ifndef __VISION_PROTOCOL_H__
#define __VISION_PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "usart.h"      // HAL 库 USART 头文件
#include "main.h"       // 包含 HAL_GetTick()

/*============================ 常量定义 ============================*/

#define VISION_SOF0             0xAA    ///< 帧头第1字节 (Start of Frame)
#define VISION_SOF1             0x55    ///< 帧头第2字节
#define VISION_MAX_FIELDS       8       ///< 单帧最大字段数
#define VISION_MAX_FRAME_LEN    32      ///< 最大帧长度（含帧头帧尾）
#define VISION_RX_BUF_SIZE      64      ///< DMA接收缓冲区大小

/*============================ 数据结构 ============================*/

/**
 * @brief 字段类型枚举 - 定义所有可能传输的数据类型
 * @note  实际传输时只传输配置表中指定的字段，未配置的字段在结构体中保持为0
 * @note  FIELD（域/成员变量）_CUSTOM0-3预留给用户自定义数据，扩展性强，无需修改协议栈代码
 */
typedef enum {
    FIELD_NONE = 0,         ///< 保留/未使用
    FIELD_VALID,            ///< 数据有效标志（0=无效，1=有效，2=丢失，3=带相位）
    FIELD_DX,               ///< 激光/目标X方向偏差（像素，有符号）
    FIELD_DY,               ///< 激光/目标Y方向偏差（像素，有符号）
    FIELD_TYPE,             ///< 目标类型（0=无，1=圆，3=三角，4=方等）
    FIELD_PHASE,            ///< 旋转相位（0-65535对应0-360°，无符号）
    FIELD_RPM,              ///< 转速（rpm，有符号，正转/反转）
    FIELD_CONFIDENCE,       ///< 置信度/亮度（0-255）
    FIELD_TIMESTAMP,        ///< 时间戳（ms，无符号32位）
    FIELD_CUSTOM0,          ///< 自定义字段0（扩展用）
    FIELD_CUSTOM1,          ///< 自定义字段1（扩展用）
    FIELD_CUSTOM2,          ///< 自定义字段2（扩展用）
    FIELD_CUSTOM3,          ///< 自定义字段3（扩展用）
    FIELD_MAX               ///< 枚举最大值（用于边界检查）
} FieldType_t;

/**
 * @brief 字段定义 - 描述单个数据字段的属性
 */
typedef struct {
    FieldType_t type;       ///< 字段类型（对应VisionPacket_t中的成员）
    uint8_t     size;       ///< 字节数（1, 2, 4）
    uint8_t     is_signed;  ///< 是否有符号（0=无符号，1=有符号）
    float       scale;      ///< 缩放因子（传输值 = 实际值 / scale）
                            ///< 例如：scale=100 表示传输值是实际值的100倍（百分之一精度）因为数据解析里不好存放浮点数的小数
} FieldDef_t;

/**
 * @brief 帧配置表 - 定义一种通信协议格式
 * @note  不同比赛/应用只需定义不同的配置表实例，无需修改代码逻辑
 */
typedef struct {
    const char* name;               ///< 配置名称（用于调试和选择）
    uint8_t     sof[2];             ///< 帧头标识（默认0xAA 0x55）
    uint8_t     field_num;          ///< 有效字段数量（不超过VISION_MAX_FIELDS）
    FieldDef_t  fields[VISION_MAX_FIELDS];  ///< 字段定义数组（按传输顺序排列）
} FrameConfig_t;

/**
 * @brief 通用视觉数据包 - 接收数据的容器
 * @note  与配置表解耦，包含所有可能字段。未配置的字段保持为0。
 */
typedef struct {
    uint8_t     valid;
    /*---- 标准视觉字段（本次电赛使用）----*/
    int16_t     dx;             ///< X方向偏差（像素）
    int16_t     dy;             ///< Y方向偏差（像素）
    uint8_t     type;           ///< 目标类型
    uint16_t    phase;          ///< 旋转相位（0-65535）
    int16_t     rpm;            ///< 转速（rpm，有符号）
    uint8_t     confidence;     ///< 置信度（激光点blob的密度/圆度作为置信度可以减少反光，两个目标靠得太近导致的判断不准）
    uint32_t    timestamp;      ///< 相机时间戳（吸取上次教训，相机传输数据到单片机需要时间，加上时间戳用于测量系统延迟和动态预测补偿）
    ///< 当相机超过某个时间没发送数据时可以判定为相机丢帧，可标志为数据无效，并且可以提前预测补偿数据，任务1，2，3可以不用
    ///<如果真的丢帧了，短期可以使用开环预测+速度保持 ，中期则慢慢减速来保护，丢帧越长速度衰减越快，直到完全停下来，等待相机恢复正常，长期则直接停机
    /*---- 扩展字段（自定义用）----*/
    int16_t     custom[4];      ///< 自定义数据0-3

    /*---- 元数据（协议栈自动填充）----*/
    uint32_t    recv_time;      ///< C板接收时间戳（HAL_GetTick()）
    uint8_t     data_valid;     ///< 本次解析是否成功（内部使用）
} VisionPacket_t;

/*============================ 外部变量 ============================*/

/**
 * @brief 全局视觉数据 - 最新接收到的数据
 * @warning 此变量在DMA中断中更新，读取时可能需要关中断保护（若需原子性）
 * @usage   直接读取：if(g_vision.valid) { 例如：float err = g_vision.dx * 0.0017f; }
 */
extern volatile VisionPacket_t g_vision;

/**
 * @brief DMA接收缓冲区 - 用户无需直接访问
 */
extern uint8_t g_vision_rx_buf[VISION_RX_BUF_SIZE];

/*============================ 配置表声明（外部定义） ============================*/

/**
 * @brief 第二次积分赛 - 13字节帧长
 * @details 字段顺序：CMD(1) + DX(2) + DY(2) + TYPE(1) + PHASE(2) + RPM(2) + CHK(1)
 * @usage   Vision_SelectConfig("QD4310");
 */
extern const FrameConfig_t g_config_QD4310;

/**
 * @brief 第一次积分赛配置 - 字段顺序：CMD(1) + DX(2) + CHK(1) = 6字节（含帧头）
 * @usage   Vision_SelectConfig("LineFollow");
 */
extern const FrameConfig_t g_config_linefollow;

/**
 * @brief 多目标扩展配置 - 带多个自定义坐标
 */
extern const FrameConfig_t g_config_multitarget;

/*============================ 函数接口 ============================*/

/**
 * @brief  初始化视觉协议栈
 * @param  huart  HAL UART句柄（连接到MaixCam的串口）
 * @note   初始化DMA接收，清空缓冲区，默认选择ESC2026配置
 * @usage  在main()初始化阶段调用一次
 */
void Vision_Init(UART_HandleTypeDef *huart);

/**
 * @brief  选择协议配置
 * @param  name  配置名称（如"QD4310", "LineFollow"）
 * @return true=找到并切换成功，false=未找到配置（保持原配置）
 * @note   切换后会自动重置接收状态机
 * @usage  Vision_SelectConfig("QD4310");
 */
bool Vision_SelectConfig(const char *name);

/**
 * @brief  UART DMA接收完成回调（必须放入HAL中断回调）
 * @param  huart  触发中断的UART句柄
 * @param  Size   本次接收到的字节数
 * @note   此函数应在HAL_UARTEx_RxEventCallback()中调用
 * @usage
 *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t size) {
 *       Vision_RxCallback(h, size);
 *   }
 */
void Vision_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief  发送命令到相机（C板->MaixCam）
 * @param  cmd    命令字节（0x01=跟踪，0x02=多目标，0x05=动态绘图等）
 * @param  param  16位参数（命令附带参数，如目标ID等）
 * @return true=发送成功
 * @note   命令帧格式：AA 55 CMD_L CMD_H PARAM_L PARAM_H 00 00 CHK（8字节）
*  @使用示例  命令0x02=多目标模式，param=3=跟踪第3个目标
             Vision_SendCommand(0x02, 3);不用参数只要发送某个任务时，param为0即可
 */
bool Vision_SendCommand(uint8_t cmd, int16_t param);

/**
 * @brief  获取当前使用的配置指针（调试用）
 * @return 当前配置表的指针
 */
const FrameConfig_t* Vision_GetCurrentConfig(void);

/**
 * @brief  计算当前配置的帧长度（调试用）
 * @return 完整帧字节数（含帧头和校验和）
 */
uint8_t Vision_GetFrameLen(void);

/**
 * @brief  手动触发数据解析（测试用）
 * @param  raw_data  原始数据缓冲区
 * @param  len       数据长度
 * @param  out       输出解析结果
 * @return true=解析成功且校验通过
 * @note   可用于解析存储的测试数据，不依赖DMA接收
 */
bool Vision_ParseManual(uint8_t *raw_data, uint16_t len, VisionPacket_t *out);

#endif /* __VISION_PROTOCOL_H__ */
