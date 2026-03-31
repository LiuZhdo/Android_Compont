/**
 * @file KYLOG_C_APIs.h
 * @author Qiyao (qiyao_chen@huace.cn)
 * @brief LOG库的C接口定义
 * @version 0.1
 * @date 2025-12-09
 *
 *
 */
#pragma once
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
#include <cstdarg>
extern "C" {
#else
#include <stdarg.h>
#include <stdio.h>
#endif

// ========== KYLOG C API (C++ KYLOG 接口的 C 语言版本) ==========

// ==================== 类型定义 ====================

/**
 * @brief 日志级别枚举
 */
typedef enum {
    KYLOG_LEVEL_VERBOSE = 0,  // 详细日志 (对应 TRACE)
    KYLOG_LEVEL_DEBUG   = 1,  // 调试日志
    KYLOG_LEVEL_INFO    = 2,  // 信息日志
    KYLOG_LEVEL_WARNING = 3,  // 警告日志
    KYLOG_LEVEL_ERROR   = 4,  // 错误日志
    KYLOG_LEVEL_FATAL   = 5   // 致命日志 (对应 CRITICAL)
} kylog_level_t;

// ==================== 全局配置接口 ====================

/**
 * @brief 获取指定标签的 Backend 类型
 * @param tag 日志标签 (文件名)
 * @return Backend 类型: 0=MMAP, 1=OTHER, 99=UNKNOWN
 */
int kylog_get_backend_type(const char *tag);

/**
 * @brief 设置默认日志输出目录  **(仅限软件上层全局操作)
 * @param log_dir 默认日志输出目录
 * @return 成功返回0, 失败返回-1
 * @note 此接口仅设置默认目录，不创建日志器
 * @warning 不允许在算法库内私自调用
 */
int kylog_set_default_dir(const char *log_dir);

/**
 * @brief 设置全局是否镜像日志到 Android 日志系统   **(仅限软件上层全局操作)
 * @param enable 是否启用镜像到 Android 日志
 * @return 成功返回0, 失败返回-1
 * @warning 不允许在算法库内私自调用
 */
int kylog_set_global_mirror_android_log(bool enable);

/**
 * @brief 刷新所有日志  **(仅限软件上层全局操作)
 * @warning 不允许在算法库内私自调用，否则影响性能
 */
void kylog_flush_all(void);

/**
 * @brief 设置日志定时刷新间隔  **(仅限软件上层全局操作)
 * @param seconds 刷新间隔时间，单位秒
 * @return 成功返回0, 失败返回-1
 * @warning 涉及系统全局性能影响，除了软件层请勿在算法库内使用
 */
int kylog_set_flush_every_time(int seconds);

// ==================== Logger 生命周期管理 ====================

/**
 * @brief 初始化指定 tag 的日志记录器，（如果该 tag 的日志器已存在,会先销毁再重新创建）
 * @param tag 日志标签(作为文件名)
 * @param log_dir 日志输出目录(如果为 NULL,则使用默认目录)
 * @param max_file_size 单个日志文件最大大小(字节),默认 10MB
 * @param max_files 最大日志文件数量(轮转),默认 3
 * @param simple_format 是否使用简化格式(仅消息内容,无时间戳等信息),默认 false
 * @return 成功返回0, 失败返回-1
 * @note 如果该 tag 的日志器已存在,会先销毁再重新创建
 * @note simple_format=true 时,日志格式为 "%v"(仅消息);false 时为标准格式
 * @note mmap backend 不支持 simple_format,该参数会被忽略
 */
int kylog_init(const char *tag, const char *log_dir, size_t max_file_size, size_t max_files, bool simple_format);

/**
 * @brief 带清除功能的初始化函数（模仿 fopen 的 w 模式）
 * @param tag 日志标签(作为文件名)
 * @param log_dir 日志输出目录(如果为 NULL,则使用默认目录)
 * @param max_file_size 单个日志文件最大大小(字节)
 * @param max_files 最大日志文件数量(轮转)
 * @param simple_format 是否使用简化格式(仅消息内容,无时间戳等信息)
 * @return 成功返回0, 失败返回-1
 * @note 此函数会：1) 检查 logger 是否存在，存在则销毁 2) 删除旧的日志文件/目录 3) 创建新的日志器
 * @warning 此操作会永久删除旧的日志文件，请谨慎使用
 * @note mmap backend 不支持 simple_format,该参数会被忽略
 */
int kylog_init_with_clear(const char *tag, const char *log_dir, size_t max_file_size, size_t max_files, bool simple_format);

/**
 * @brief 销毁指定标签的日志器
 * @param tag 日志标签 (文件名)
 */
void kylog_destroy(const char *tag);

/**
 * @brief 销毁指定标签的日志器并删除对应的日志文件/目录
 * @param tag 日志标签 (文件名)
 * @return 成功返回0, 失败返回-1
 * @note 会先销毁日志器(释放文件句柄), 然后删除对应的日志目录
 * @warning 此操作不可逆，会永久删除日志文件，请谨慎使用
 */
int kylog_destroy_and_delete(const char *tag);

/**
 * @brief 刷新指定标签的日志
 * @param tag 日志标签 (文件名)
 */
void kylog_flush(const char *tag);

// ==================== Logger 配置接口 ====================

/**
 * @brief 设置指定 tag 是否镜像日志到 Android 日志系统
 * @param tag 日志标签（文件名）
 * @param enable 是否启用镜像到 Android 日志
 * @return 成功返回0, 失败返回-1
 * @note 如果 tag 为 NULL，则设置全局默认值（对后续新创建的 logger 生效）
 */
int kylog_set_mirror_to_android_log(const char *tag, bool enable);

/**
 * @brief 设置指定 tag 的日志格式
 * @param tag 日志标签(文件名)
 * @param pattern 格式字符串,例如:
 *                "[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v" (标准格式)
 *                "%v" (仅消息内容,简化格式,用于性能测试)
 * @return 成功返回0, 失败返回-1
 * @note 必须在 kylog_init 之后调用
 * @note mmap backend 不支持此功能
 */
int kylog_set_pattern(const char *tag, const char *pattern);

/**
 * @brief 设置指定 tag 的最小日志级别(过滤功能)
 * @param tag 日志标签(文件名)
 * @param level 最小日志级别
 * @return 成功返回0, 失败返回-1
 * @note 只有大于等于此级别的日志才会被记录
 * @note 运行时动态调整,无锁实现
 */
int kylog_set_min_level(const char *tag, kylog_level_t level);

/**
 * @brief 设置指定 tag 的自动刷新间隔时间
 * @param tag 日志标签(文件名)
 * @param seconds 刷新间隔时间，单位秒 (必须大于0)
 * @return 成功返回0, 失败返回-1
 * @note 每隔指定秒数自动刷新日志缓冲区
 * @note 运行时动态调整,立即生效
 */
int kylog_set_flush_interval(const char *tag, int seconds);

// ==================== 日志写入接口 ====================

/**
 * @brief 通用日志接口（支持自定义级别）
 * @param tag 日志标签 (作为文件名)
 * @param level 日志级别
 * @param fmt printf风格的格式字符串
 */
void kylog_log(const char *tag, kylog_level_t level, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

/**
 * @brief VERBOSE级别日志 (对应 TRACE)
 * @param tag 日志标签 (作为文件名)
 * @param fmt printf风格的格式字符串
 */
void kylog_v(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/**
 * @brief DEBUG级别日志
 * @param tag 日志标签 (作为文件名)
 * @param fmt printf风格的格式字符串
 */
void kylog_d(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/**
 * @brief INFO级别日志
 * @param tag 日志标签 (作为文件名)
 * @param fmt printf风格的格式字符串
 */
void kylog_i(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/**
 * @brief WARNING级别日志
 * @param tag 日志标签 (作为文件名)
 * @param fmt printf风格的格式字符串
 */
void kylog_w(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/**
 * @brief ERROR级别日志
 * @param tag 日志标签 (作为文件名)
 * @param fmt printf风格的格式字符串
 */
void kylog_e(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/**
 * @brief FATAL级别日志 (对应 CRITICAL)
 * @param tag 日志标签 (作为文件名)
 * @param fmt printf风格的格式字符串
 */
void kylog_f(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

#ifdef __cplusplus
}
#endif // __cplusplus

