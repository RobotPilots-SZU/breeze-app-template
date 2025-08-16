#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

// 注册日志
LOG_MODULE_REGISTER(app);

// 获取LED设备
#define LED_STRIP_NODE DT_NODELABEL(rgb_led)
static const struct device *const led_strip = DEVICE_DT_GET(LED_STRIP_NODE);

int main(void)
{
	// 检查设备是否就绪
	if (!device_is_ready(led_strip)) {
		printk("LED strip device %s is not ready\n", led_strip->name);
		return -1; // 必须返回整数值
	}

	// 使用Zephyr的标准led_rgb结构体
	struct led_rgb colors[1] = {
		{.r = 0x00, .g = 0x00, .b = 0x00}, // 初始黑色（关闭）
	};
   
	while (1) {
		// 红色
		colors[0].r = 0xFF;
		colors[0].g = 0x00;
		colors[0].b = 0x00;
		led_strip_update_rgb(led_strip, colors, 1);
		k_sleep(K_MSEC(1000));
		printk("red now\r\n");

		// 绿色
		colors[0].r = 0x00;
		colors[0].g = 0xFF;
		colors[0].b = 0x00;
		led_strip_update_rgb(led_strip, colors, 1);
		k_sleep(K_MSEC(1000));
		printk("green now\r\n");

		// 蓝色
		colors[0].r = 0x00;
		colors[0].g = 0x00;
		colors[0].b = 0xFF;
		led_strip_update_rgb(led_strip, colors, 1);
		k_sleep(K_MSEC(1000));
		printk("blue now\r\n");

		LOG_INF("a loop complicate");
	}

	return 0; // 虽然不会执行到这里，但保持语法完整
}
