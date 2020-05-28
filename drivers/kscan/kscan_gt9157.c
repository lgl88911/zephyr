/*
 * Copyright (c) 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/kscan.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(gt9157, CONFIG_KSCAN_LOG_LEVEL);

#define GT9157_I2C_ADDRESS_A	0x14
#define GT9157_I2C_ADDRESS_B	0x5D

#define GT9157_COOR_ADDR    0x814E
#define GT9157_REG_SLEEP         0x8040
#define GT9157_REG_SENSOR_ID     0x814A
#define GT9157_REG_CONFIG_DATA   0x8047
#define GT9157_REG_VERSION       0x8140

#define GT9157_REG_NUMBER	240
#define GT9157_REG_ADDR_LEN	2

#define GT9157_VERSION_LEN		6

#define GT9157_MAX_TOUCH	5
#define	GT9157_PER_TOUCH_LEN	8
#define GT9157_TOUCH_STATUS_LEN	1
#define GT9157_TOUCH_RESERVED_LEN	1

static uint8_t gt9157_cfg_regs[] ={
	0x00,0x20,0x03,0xE0,0x01,0x05,0x3C,0x00,0x01,0x08,
	0x28,0x0C,0x50,0x32,0x03,0x05,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x17,0x19,0x1E,0x14,0x8B,0x2B,0x0D,
	0x33,0x35,0x0C,0x08,0x00,0x00,0x00,0x9A,0x03,0x11,
	0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x32,0x00,0x00,
	0x00,0x20,0x58,0x94,0xC5,0x02,0x00,0x00,0x00,0x04,
	0xB0,0x23,0x00,0x93,0x2B,0x00,0x7B,0x35,0x00,0x69,
	0x41,0x00,0x5B,0x4F,0x00,0x5B,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,
	0x12,0x14,0x16,0x18,0x1A,0xFF,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0F,
	0x10,0x12,0x13,0x16,0x18,0x1C,0x1D,0x1E,0x1F,0x20,
	0x21,0x22,0x24,0x26,0xFF,0xFF,0xFF,0xFF,0x00,0x00,
	0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0x48,0x01
};

struct GT9157_config {
	char *i2c_name;
	u8_t i2c_address;
	char *int_port ;
	u32_t int_pin;
	u32_t int_flags;
	char *rst_port;
	u32_t rst_pin;
	u32_t rst_flags;
};

struct GT9157_data {
	struct device *i2c;
	struct device *gpio_int;
	struct device *gpio_rst;
	kscan_callback_t callback;
	struct k_work work;
	struct k_timer timer;
	struct device *dev;
};

struct tunch_point_info{
	u8_t id;
	u16_t x;
	u16_t y;
	u16_t size;
};

static int gt9157_write_regs(struct device *dev, u8_t dev_addr,
	u16_t reg_addr,	u8_t *regs, u16_t reg_num)
{
	u8_t buf[GT9157_REG_ADDR_LEN + GT9157_REG_NUMBER] =
		{reg_addr >> 8, reg_addr & 0xFF,};

	if(reg_num > GT9157_REG_NUMBER){
		return -1;
	}
	memcpy(&buf[GT9157_REG_ADDR_LEN], regs, reg_num);

	return i2c_write(dev, buf, reg_num+GT9157_REG_ADDR_LEN, dev_addr);
}


static int gt9157_read_regs(struct device *dev, u8_t dev_addr,
	u16_t reg_addr,	u8_t *regs, u16_t reg_num)
{
	u8_t addr[2] = {reg_addr >> 8, reg_addr & 0xFF};
	return i2c_write_read(dev, dev_addr, addr, 2, regs, reg_num);
}

static void gt9157_cb(struct device *dev,
				  struct gpio_callback *cb, u32_t pins)
{
	printk("touch int\n");
}

static int GT9157_read(struct device *dev)
{
	const struct GT9157_config *config = dev->config->config_info;
	struct GT9157_data *data = dev->driver_data;
	u8_t buf[GT9157_PER_TOUCH_LEN*GT9157_MAX_TOUCH];
	int ret = 0;
	u8_t status = 0;
	u8_t i,j;
	u8_t touch_number = 0;
	struct tunch_point_info cur_touch[GT9157_MAX_TOUCH] = {0};

	static u8_t pre_touch_number = 0;
	static struct tunch_point_info pre_touch[GT9157_MAX_TOUCH] = {0};

	//read touch status
	ret = gt9157_read_regs(data->i2c, config->i2c_address,
		GT9157_COOR_ADDR, &status, GT9157_TOUCH_STATUS_LEN);
	if (ret) {
		LOG_ERR("Could not read point");
		return -EIO;
	}

	printk("read %x\n", status);

	/* No data */
	if(status == 0){
		return 0;
	}

	do{
		//data no ready
		if((status & 0x80) == 0){
		  break;
		}

		touch_number = status & 0x0f;
		if(touch_number > GT9157_MAX_TOUCH){
			break;
		}

		ret = gt9157_read_regs(data->i2c, config->i2c_address,
			GT9157_COOR_ADDR+GT9157_TOUCH_STATUS_LEN,
			buf, touch_number*GT9157_PER_TOUCH_LEN);

		if (ret) {
			LOG_ERR("Could not read point");
			ret = -EIO;
			break;
		}

		//check press key
		for(i=0; i<touch_number; i++){
			cur_touch[i].id = buf[i*GT9157_PER_TOUCH_LEN];
			cur_touch[i].x = buf[i*GT9157_PER_TOUCH_LEN+2];
			cur_touch[i].x = (cur_touch[i].x<<8) | buf[i*GT9157_PER_TOUCH_LEN+1];
			cur_touch[i].y = buf[i*GT9157_PER_TOUCH_LEN+4];
			cur_touch[i].y = (cur_touch[i].x<<8) | buf[i*GT9157_PER_TOUCH_LEN+3];
			cur_touch[i].size = buf[i*GT9157_PER_TOUCH_LEN+2];
			cur_touch[i].size = (cur_touch[i].x<<6) | buf[i*GT9157_PER_TOUCH_LEN+5];

			for(j=0; j<pre_touch_number; j++){
				if(cur_touch[i].id == pre_touch[i].id){
					break;
				}
			}

			if(j>=pre_touch_number){
				data->callback(dev, cur_touch[i].x, cur_touch[i].y, true);
			}
		}

		//check release key
		for(i=0; i<pre_touch_number; i++){
			for(j=0; j<touch_number; j++){
				if(cur_touch[i].id == pre_touch[i].id){
					break;
				}
			}

			if(j>=touch_number){
				data->callback(dev, pre_touch[i].x, pre_touch[i].y, false);
			}
		}

		memcpy(pre_touch, cur_touch, sizeof(cur_touch));
		pre_touch_number = touch_number;
	}while(0);

	status = 0;
	gt9157_write_regs(data->i2c, config->i2c_address,
		GT9157_COOR_ADDR, &status, GT9157_TOUCH_STATUS_LEN);

	return ret;
}

static void GT9157_timer_handler(struct k_timer *timer)
{
	struct GT9157_data *data =
		CONTAINER_OF(timer, struct GT9157_data, timer);

	k_work_submit(&data->work);
}

static void GT9157_work_handler(struct k_work *work)
{
	struct GT9157_data *data =
		CONTAINER_OF(work, struct GT9157_data, work);

	GT9157_read(data->dev);
}

static int GT9157_configure(struct device *dev, kscan_callback_t callback)
{
	struct GT9157_data *data = dev->driver_data;

	if (!callback) {
		return -EINVAL;
	}

	data->callback = callback;

	return 0;
}

static int GT9157_enable_callback(struct device *dev)
{
	struct GT9157_data *data = dev->driver_data;

	k_timer_start(&data->timer, K_MSEC(CONFIG_KSCAN_GT9157_PERIOD),
		      K_MSEC(CONFIG_KSCAN_GT9157_PERIOD));

	return 0;
}

static int GT9157_disable_callback(struct device *dev)
{
	struct GT9157_data *data = dev->driver_data;

	k_timer_stop(&data->timer);

	return 0;
}

static int GT9157_init(struct device *dev)
{
	const struct GT9157_config *config = dev->config->config_info;
	struct GT9157_data *data = dev->driver_data;
    struct gpio_callback kscan_cb;
	u8_t version[GT9157_VERSION_LEN];
	int ret = -1;
	u8_t check_sum;
	u16_t reg_num;

#ifdef DT_INST_0_COODIX_GT9157_INT_GPIOS_CONTROLLER
	data->gpio_int = device_get_binding(config->int_port);
	if (data->gpio_int == NULL) {
		LOG_ERR("Could not find INT GPIO device");
		return -EINVAL;
	}
#endif

#ifdef DT_INST_0_COODIX_GT9157_RESET_GPIOS_CONTROLLER
	data->gpio_rst = device_get_binding(config->rst_port);
	if (data->gpio_rst == NULL) {
		LOG_ERR("Could not find rst GPIO device");
		return -EINVAL;
	}
#endif

	/* configure i2c address*/
	if (data->gpio_int != NULL && data->gpio_rst != NULL) {
		if (gpio_pin_configure(data->gpio_int, config->int_pin,
			       GPIO_OUTPUT | config->int_flags)) {
            LOG_ERR("Unable to configure int GPIO pin %u", config->int_pin);
            return -EINVAL;
	    }

		if (gpio_pin_configure(data->gpio_rst, config->rst_pin,
			       GPIO_OUTPUT | config->rst_flags)) {
            LOG_ERR("Unable to configure rst GPIO pin %u", config->rst_pin);
            return -EINVAL;
	    }

        gpio_pin_set(data->gpio_rst, config->rst_pin, 0);
		gpio_pin_set(data->gpio_int, config->int_pin, 0);
		k_sleep(K_MSEC(1));

		if(config->i2c_address == GT9157_I2C_ADDRESS_A) {
            gpio_pin_set(data->gpio_int, config->int_pin, 1);
		} else if (config->i2c_address == GT9157_I2C_ADDRESS_B) {
            gpio_pin_set(data->gpio_int, config->int_pin, 0);
		} else {
			LOG_ERR("No support I2C address %02x", config->i2c_address);
			return -EINVAL;
		}

        k_sleep(K_MSEC(1));
        gpio_pin_set(data->gpio_rst, config->rst_pin, 1);

        k_sleep(K_MSEC(6));
	}

    if(data->gpio_int != NULL){
        gpio_pin_configure(data->gpio_int, config->int_pin, GPIO_INPUT | config->int_flags);
        gpio_init_callback(&kscan_cb, gt9157_cb, BIT(config->int_pin));
        gpio_add_callback(data->gpio_int, &kscan_cb);
    }

	data->i2c = device_get_binding(config->i2c_name);
	if (data->i2c == NULL) {
		LOG_ERR("Could not find I2C device");
		return -EINVAL;
	}

	ret = gt9157_read_regs(data->i2c,config->i2c_address,
		GT9157_REG_VERSION, version, GT9157_VERSION_LEN);

	if (ret != 0) {
		LOG_ERR("I2C read reg fail\n");
		return -EINVAL;
	}

	if(version[0] != '9' || version[1] != '1' || version[2] != '5' || version[3] != '7'){
		LOG_ERR("GT9157 Version no match\n");
		//return -EINVAL;
	}

	reg_num = sizeof(gt9157_cfg_regs);
	check_sum = 0;
	for (int i = 0; i < reg_num; i++)
	{
		check_sum += gt9157_cfg_regs[i];
	}

	u8_t buf[reg_num+2];
	memcpy(buf, gt9157_cfg_regs, reg_num);
	buf[reg_num] = (~check_sum) + 1;
	buf[reg_num+1] =  1;
	reg_num += 2;

	ret = gt9157_write_regs(data->i2c,config->i2c_address,
		GT9157_REG_CONFIG_DATA, buf, reg_num);

	if (ret != 0) {
		LOG_ERR("confiure gt9157 fail\n");
		return -EINVAL;
	}

	k_sleep(K_MSEC(50));


	data->dev = dev;

	k_work_init(&data->work, GT9157_work_handler);
	k_timer_init(&data->timer, GT9157_timer_handler, NULL);

	GT9157_enable_callback(dev);

	return 0;
}


static const struct kscan_driver_api GT9157_driver_api = {
	.config = GT9157_configure,
	.enable_callback = GT9157_enable_callback,
	.disable_callback = GT9157_disable_callback,
};

static const struct GT9157_config GT9157_config = {
	.i2c_name = DT_INST_0_COODIX_GT9157_BUS_NAME,
	.i2c_address = DT_INST_0_COODIX_GT9157_BASE_ADDRESS,
	.int_port = DT_INST_0_COODIX_GT9157_INT_GPIOS_CONTROLLER,
	.int_pin = DT_INST_0_COODIX_GT9157_INT_GPIOS_PIN,
	.int_flags = DT_INST_0_COODIX_GT9157_INT_GPIOS_FLAGS,
	.rst_port = DT_INST_0_COODIX_GT9157_RESET_GPIOS_CONTROLLER,
	.rst_pin = DT_INST_0_COODIX_GT9157_RESET_GPIOS_PIN,
	.rst_flags = DT_INST_0_COODIX_GT9157_RESET_GPIOS_FLAGS,
};

static struct GT9157_data GT9157_data;

DEVICE_AND_API_INIT(GT9157, DT_INST_0_COODIX_GT9157_LABEL, GT9157_init,
		    &GT9157_data, &GT9157_config,
		    POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,
		    &GT9157_driver_api);
