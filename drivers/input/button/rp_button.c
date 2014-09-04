#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

#include <linux/io.h>
#include <mach/regs-clock.h>


static struct timer_list timer;
static struct input_dev * input;

struct st_s3c_key{
	int key_code;
	uint key_pin;
	int key_history_flg;
};

static struct st_s3c_key s3c_key_para[] = {
		{ KEY_BACK, EXYNOS4_GPX3(3), 0},
		{ KEY_MENU, EXYNOS4_GPX3(2), 0},
		{ KEY_HOME, EXYNOS4_GPX3(1), 0},
		{ KEY_VOLUMEUP, EXYNOS4_GPX2(6), 0},
		{ KEY_VOLUMEDOWN, EXYNOS4_GPX2(7), 0},	
		{ KEY_UP, EXYNOS4_GPX2(4), 0},
		{ KEY_DOWN, EXYNOS4_GPX2(5), 0},
		{ KEY_LEFT, EXYNOS4_GPX2(0), 0},
		{ KEY_F13, EXYNOS4_GPX2(1), 0},		
};

#define MAX_BUTTON_CNT 	ARRAY_SIZE(s3c_key_para)


static int s3c_Keycode[MAX_BUTTON_CNT];

static void s3cbutton_timer_handler(unsigned long data)
{
	int flag;
	int i;
	for(i=0; i<MAX_BUTTON_CNT; i++) 
	{
		flag = gpio_get_value(s3c_key_para[i].key_pin);
		if(flag != s3c_key_para[i].key_history_flg) 
		{
			if(flag) 
			{
				input_report_key(input, s3c_key_para[i].key_code, 0);
				//printk("s3c-button back key up!\n");
 			}  
			else  
			{
				input_report_key(input, s3c_key_para[i].key_code, 1);
				//printk("s3c-button back key down!!\n");
			}
			s3c_key_para[i].key_history_flg= flag;

			input_sync(input);
		}
	}

	/* Kernel Timer restart */
	mod_timer(&timer,jiffies + HZ/20);
}


static int s3c_button_probe(struct platform_device *pdev)
{
	int i;

	for(i=0; i<MAX_BUTTON_CNT; i++)  {
		gpio_request(s3c_key_para[i].key_pin, "s3c-button");
		s3c_gpio_setpull(s3c_key_para[i].key_pin, S3C_GPIO_PULL_UP);
		gpio_direction_input(s3c_key_para[i].key_pin);
		s3c_Keycode[i]=s3c_key_para[i].key_code;
	}

	input = input_allocate_device();
	if(!input) 
		return -ENOMEM;

	set_bit(EV_KEY, input->evbit);
	//set_bit(EV_REP, input->evbit);	/* Repeat Key */

	for(i = 0; i < MAX_BUTTON_CNT; i++)
		set_bit(s3c_key_para[i].key_code, input->keybit);

	input->name = "s3c-button";
	input->phys = "s3c-button/input0";

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	input->keycode = s3c_Keycode;
	input->keycodemax  = ARRAY_SIZE(s3c_Keycode);
	input->keycodesize = sizeof(s3c_Keycode[0]);

	if(input_register_device(input) != 0)
	{
		printk("s3c-button input register device fail!!\n");

		input_free_device(input);
		return -ENODEV;
	}

	/* Scan timer init */
	init_timer(&timer);
	timer.function = s3cbutton_timer_handler;

	timer.expires = jiffies + (HZ/20);
	add_timer(&timer);

	printk("s3c button Initialized!!\n");

	return 0;
}

static int s3c_button_remove(struct platform_device *pdev)
{
	int i;

	input_unregister_device(input);
	del_timer(&timer);
	for(i=0; i<MAX_BUTTON_CNT; i++)  {
		gpio_free(s3c_key_para[i].key_pin);
	}
	return  0;
}


#ifdef CONFIG_PM
static int s3c_button_suspend(struct platform_device *pdev, pm_message_t state)
{
       printk("s3c_button_suspend!!\n");
	return 0;
}

static int s3c_button_resume(struct platform_device *pdev)
{
	printk("s3c_button_resume!!\n");
	return 0;
}
#else
#define s3c_button_suspend	NULL
#define s3c_button_resume	NULL
#endif

static struct platform_driver s3c_button_device_driver = {
	.probe		= s3c_button_probe,
	.remove		= s3c_button_remove,
	.suspend	= s3c_button_suspend,
	.resume		= s3c_button_resume,
	.driver		= {
		.name	= "s3c-button",
		.owner	= THIS_MODULE,
	}
};


static struct platform_device s3c_device_button = {
	.name	= "s3c-button",
	.id		= -1,
};

static int __init s3c_button_init(void)
{
	platform_device_register(&s3c_device_button);
	return platform_driver_register(&s3c_button_device_driver);
}

static void __exit s3c_button_exit(void)
{
	platform_driver_unregister(&s3c_button_device_driver);
	platform_device_unregister(&s3c_device_button);
}

module_init(s3c_button_init);
module_exit(s3c_button_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("rongpin");
MODULE_DESCRIPTION("Keyboard driver for s3c button.");
MODULE_ALIAS("platform:s3c-button");
