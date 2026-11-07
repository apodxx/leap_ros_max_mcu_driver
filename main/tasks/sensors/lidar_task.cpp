#include "system_globals.h"
#include "board_config.h"
#include "camsense_lidar.h"
#include "network_config.h"
#include "msg/lidar_msg.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "LIDAR_TASK";
namespace board_cfg = leap_config::board;
namespace net_cfg = leap_config::network;

void lidar_task(void *p) {
    CamsenseLidar* lidar = new CamsenseLidar(
        net_cfg::kLidarUartPort,
        board_cfg::kLidarTxPin,
        board_cfg::kLidarRxPin,
        net_cfg::kLidarBaudRate
    );

    lidar->Init();

    static LidarPoint full_scan_data[MAX_LIDAR_POINTS_PER_SCAN];
    uint16_t scan_point_count = 0;
    LidarMsg lidar_msg;

    ESP_LOGI(TAG, "Camsense lidar task started on UART%d, RX=%d, baud=%d",
             static_cast<int>(net_cfg::kLidarUartPort),
             static_cast<int>(board_cfg::kLidarRxPin),
             net_cfg::kLidarBaudRate);

    while (1) {
        if (lidar->Poll(full_scan_data, &scan_point_count)) {
            memset(&lidar_msg, 0, sizeof(lidar_msg));

            for (int i = 0; i < scan_point_count; i++) {
                // Camsense 的 0° 方向与车体安装方向相差 180°，做镜像。
                int angle_deg = static_cast<int>(full_scan_data[i].angle + 0.5f);
                if (angle_deg >= 360) angle_deg -= 360;
                if (angle_deg < 0) angle_deg = 0;
                angle_deg = (360 - angle_deg) % 360;

                uint16_t dist = static_cast<uint16_t>(full_scan_data[i].distance);
                if (dist == 0) {
                    continue;
                }
                if (lidar_msg.distances[angle_deg] == 0 ||
                    dist < lidar_msg.distances[angle_deg]) {
                    lidar_msg.distances[angle_deg] = dist;
                }
            }
            xQueueOverwrite(system_globals().queues.lidar_state, &lidar_msg);
            system_globals().flags.lidar_scan_sequence =
                system_globals().flags.lidar_scan_sequence + 1;
        }

        vTaskDelay(pdMS_TO_TICKS(net_cfg::kLidarTaskPeriodMs));
    }
}
