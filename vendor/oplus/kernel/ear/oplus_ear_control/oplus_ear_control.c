/*

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

   This program is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.


   Copyright (C) 2006-2007 - Motorola
   Copyright (c) 2008 QUALCOMM USA, INC.

   Date         Author           Comment
   -----------  --------------   --------------------------------
   2006-Apr-28	Motorola	 The kernel module for running the Bluetooth(R)
				 Sleep-Mode Protocol from the Host side
   2006-Sep-08  Motorola         Added workqueue for handling sleep work.
   2007-Jan-24  Motorola         Added mbm_handle_ioi() call to ISR.

*/

#define pr_fmt(fmt) "ear_control: %s: " fmt, __func__
#include <linux/gpio.h>
#include <linux/module.h>	/* kernel module definitions */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <net/genetlink.h>

#include <linux/irq.h>
#include <linux/param.h>
#include <linux/bitops.h>
#include <linux/termios.h>


#define BT_ERR(fmt, ...)    \
    pr_err(fmt, ##__VA_ARGS__)

#define BT_WARN(fmt, ...)    \
	pr_warn(fmt, ##__VA_ARGS__)
#define BT_INFO(fmt, ...)     \
    pr_info(fmt, ##__VA_ARGS__)
#define BT_DBG( fmt, ...)     \
    pr_info(fmt, ##__VA_ARGS__)

/*
 * Defines
 */

#define VERSION		"1.0"
#define PROC_DIR	"ear_control/sleep"

#define PROC_BTWAKE	0
#define PROC_HOSTWAKE	1
#define PROC_WAKEUPHOSTENABLE	2
#define PROC_ASLEEP	3

#define PROC_RSTENABLE	4


/* state variable names and bit positions */
#define BT_WAKEUPHOST_ENABLE	0x01            //使能 ear wake up host wake irq
#define BT_TXDATA	0x02            //有数据需要发送给BT
#define BT_ASLEEP	0x04            //BT处于sleep状态
#define BT_EXT_WAKE	0x08            //BT退出wake状态            1.ext wake 0.wake bt
#define BT_SUSPEND	0x10            //BT处于suspend状态
#define BT_RSTENABLE 0x20            //RST处于enable状态


#define BT_PORT_ID  1

#define EAR_NETLINK_GROUP_ID  5

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 1,
	DEBUG_BTWAKE = 1U << 2,
	DEBUG_VERBOSE = 1U << 3,
};
static int debug_mask = DEBUG_USER_STATE | DEBUG_BTWAKE | DEBUG_VERBOSE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

struct bluesleep_info {
    struct device *dev;
	struct gpio_desc *host_wake;         //1.唤醒host,读GPIO_176,如果为高则需要唤醒HOST,如果为低则允许HOST休眠
	struct gpio_desc *ear_wake;          //1.拉高唤醒B3,拉低允许B3睡眠
    struct gpio_desc *ear_rst;
	int host_wake_irq;     //bt触发中断唤醒host
	int ear_rst_irq;
	struct uart_port *uport;
	bool isFT;
};

/* work function */
static void ear_control_sleep_work(struct work_struct *work);

/* work queue */
DECLARE_DELAYED_WORK(sleep_workqueue, ear_control_sleep_work);

/* Macros for handling sleep work */
#define ear_control_rx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define ear_control_tx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define ear_control_rx_idle()     schedule_delayed_work(&sleep_workqueue, 0)
#define ear_control_tx_idle()     schedule_delayed_work(&sleep_workqueue, 0)

/* 1 second timeout */
#define TX_TIMER_INTERVAL	1



/* global pointer to a single hci device. */

static struct bluesleep_info *bsi;

/* module usage */
static atomic_t open_count = ATOMIC_INIT(1);


/*
 * Global variables
 */

/** Global state flags */
static unsigned long flags;

/** Tasklet to respond to change in hostwake line */
static struct tasklet_struct hostwake_task;
static struct tasklet_struct ear_rst_task;


/** Transmission timer */
static struct timer_list tx_timer;

/** Lock for state transitions */
static spinlock_t rw_lock;


struct proc_dir_entry *bluetooth_dir, *sleep_dir;

static struct platform_driver ear_control_driver;
static struct genl_family ear_control_family;

enum ear_multicast_groups {
	EAR_CONTROL_MCGRP_EVENTS = 0,
};


enum ear_netlink_cmd {
    EAR_NETLINK_GPIO_NOTIFY = 0,
};

enum ear_netlink_attr {
    EAR_ATTR_GPIO_TYPE = 0,
};

enum ear_gpio_type {
    EAR_GPIO_WAKE_UP_HOST = 0,
    EAR_GPIO_EAR_RST,
};


/*
 * Local functions
 */

static unsigned int msm_hs_tx_empty(struct uart_port *uport)
{
	return 1;
}


static int ear_fill_gpio_notify(struct sk_buff *skb, uint16_t gpio_type)
{
    void *hdr;
    
    hdr = genlmsg_put(skb, 0, 0, &ear_control_family, 0, EAR_NETLINK_GPIO_NOTIFY);
    if (hdr == NULL)
		return -EMSGSIZE;
    
    if (nla_put_u16(skb, EAR_ATTR_GPIO_TYPE, gpio_type)) {
        goto nl_put_failure;
    }
    	
	genlmsg_end(skb, hdr);
	return 0;
nl_put_failure:
    genlmsg_cancel(skb, hdr);
    return -EMSGSIZE;

    
}

static void ear_gpio_notify(uint16_t gpio_type)
{
    struct sk_buff *msg;
    msg = nlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg)
		return;
	if (ear_fill_gpio_notify(msg, gpio_type)) {
		nlmsg_free(msg);
		return;
	}	
	genlmsg_multicast_allns(&ear_control_family, msg, 0, EAR_CONTROL_MCGRP_EVENTS, GFP_KERNEL);
	return;
}

static inline void ear_control_ear_wake()
{
    clear_bit(BT_EXT_WAKE, &flags);
    gpiod_set_value(bsi->ear_wake, 1);
    return;
}

static inline void ear_control_ear_sleep()
{
    set_bit(BT_EXT_WAKE, &flags);
    gpiod_set_value(bsi->ear_wake, 0);
    return;
}


/**
 * @return 1 if the Host can go to sleep, 0 otherwise.
 */
static inline int ear_control_can_sleep(void)
{
	/* check if MSM_WAKE_BT_GPIO and BT_WAKE_MSM_GPIO are both deasserted */
    /*
	return !gpiod_get_value(bsi->ear_wake) &&
		!gpiod_get_value(bsi->host_wake) &&
		(bsi->uport != NULL);
	*/
	
	return !gpiod_get_value(bsi->ear_wake) &&
		!gpiod_get_value(bsi->host_wake);
}

		/* Start the timer */
		

/**
 * @brief@  main sleep work handling function which update the flags
 * and activate and deactivate UART ,check FIFO.
 */
static void ear_control_sleep_work(struct work_struct *work)
{
	if (ear_control_can_sleep()) {
		/* already asleep, this is an error case */
		if (test_bit(BT_ASLEEP, &flags)) {
			BT_INFO("already asleep");
			return;
		}

		if (msm_hs_tx_empty(bsi->uport)) {
			BT_INFO("going to sleep...");
			set_bit(BT_ASLEEP, &flags);
			/*Deactivating UART */
			del_timer(&tx_timer);
			pm_wakeup_dev_event(bsi->dev, HZ / 2, false);
		} else {
			mod_timer(&tx_timer, (jiffies + (TX_TIMER_INTERVAL * HZ)));
			return;
		}
	} else {
	    BT_INFO("no ready to sleep...");
	    mod_timer(&tx_timer, (jiffies + (TX_TIMER_INTERVAL * HZ)));
	}
}

/**
 * A tasklet function that runs in tasklet context and reads the value
 * of the HOST_WAKE GPIO pin and further defer the work.
 * @param data Not used.
 */
static void ear_control_hostwake_task(unsigned long data)
{
    BT_INFO("Recive the irq wake up host from B3");
    ear_gpio_notify(EAR_GPIO_WAKE_UP_HOST);
    //ear_wake_host_uevent_notify();
	spin_lock(&rw_lock);
	if (gpiod_get_value(bsi->host_wake)) {
		ear_control_rx_busy();
	} else {
		ear_control_rx_idle();
    }
	spin_unlock(&rw_lock);
}
static void ear_control_rst_task(unsigned long data)
{
    BT_INFO("Recive the irq ear rst from B3");
    //ear_wake_host_uevent_notify();

    ear_gpio_notify(EAR_GPIO_EAR_RST);
    pm_relax(bsi->dev);
}

/**
 * Handles transmission timer expiration.
 * @param data Not used.
 */
static void ear_control_tx_timer_expire(struct timer_list *timer)
{
	BT_INFO("Tx timer expired");
	
    ear_control_tx_busy();
}

/**
 * Schedules a tasklet to run when receiving an interrupt on the
 * <code>HOST_WAKE</code> GPIO pin.
 * @param irq Not used.
 * @param dev_id Not used.
 */
static irqreturn_t ear_control_hostwake_isr(int irq, void *dev_id)
{
	//gpio_clear_detect_status(bsi->host_wake_irq);
	/* schedule a tasklet to handle the change in the host wake line */
    /* keep android wake, Prevent android from going to sleep on interrupt bottom half */
    pm_stay_awake(bsi->dev);
    mod_timer(&tx_timer, (jiffies + (TX_TIMER_INTERVAL * HZ)));
	tasklet_schedule(&hostwake_task);
	return IRQ_HANDLED;
}

static void ear_control_enable_irq(int irq, bool isFT)
{
    if (enable_irq_wake(irq) < 0) {
        BT_ERR("Couldn't enable %d as wakeup interrupt", irq);
        return;
    }
    if (isFT) {
        enable_irq(irq);
    }
}
static void ear_control_disable_irq(int irq, bool isFT)
{
    if (disable_irq_wake(irq))
        BT_ERR("Couldn't disable %d wakeup mode \n", irq);
    if (isFT) {
        disable_irq(irq);
    }
}

/**
 * Starts the Sleep-Mode Protocol on the Host.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int ear_control_enable_ear_wakeup_host(bool isFt)
{
	//int retval;
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);
    /* already in sleep mode */
	if (test_bit(BT_WAKEUPHOST_ENABLE, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return 0;
	}
	
	if (!atomic_dec_and_test(&open_count)) {
		atomic_inc(&open_count);
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		BT_ERR("BT WAKE: invalid order of operations");
		return -EBUSY;
	}
	bsi->isFT = isFt;
	/* Set host start sleep flag */
	set_bit(BT_WAKEUPHOST_ENABLE, &flags);
	spin_unlock_irqrestore(&rw_lock, irq_flags);
    if (debug_mask & DEBUG_BTWAKE)
		BT_INFO("enable the wakeup host irq. FT: %d", isFt);
    
    ear_control_enable_irq(bsi->host_wake_irq, isFt);
    
	return 0;
}

/**
 * Stops the Sleep-Mode Protocol on the Host.
 */
static void ear_control_disable_ear_wakeup_host(bool isFt)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);

	if (!test_bit(BT_WAKEUPHOST_ENABLE, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return;
	}

    if (debug_mask & DEBUG_BTWAKE)
		BT_INFO("disable the wakeup host irq. FT: %d", isFt);
	
	del_timer(&tx_timer);
	clear_bit(BT_WAKEUPHOST_ENABLE, &flags);

    clear_bit(BT_ASLEEP, &flags);
	atomic_inc(&open_count);

	spin_unlock_irqrestore(&rw_lock, irq_flags);
	ear_control_disable_irq(bsi->host_wake_irq, isFt);

	pm_relax(bsi->dev);
}

static int ear_control_gpio_request(struct device *dev)
{
    bsi->host_wake = devm_gpiod_get(dev, "host_wake", GPIOD_IN);
    if (IS_ERR(bsi->host_wake)) {
		BT_ERR("failed to acquire host wake gpio");
		return PTR_ERR(bsi->host_wake);
	}
	
	bsi->ear_rst = devm_gpiod_get(dev, "ear_rst", GPIOD_IN);
    if (IS_ERR(bsi->ear_rst)) {
		BT_ERR("failed to acquire ear rst gpio");
		return PTR_ERR(bsi->ear_rst);
	}
	
	bsi->ear_wake = devm_gpiod_get(dev, "ear_wake", GPIOD_OUT_LOW);
	return PTR_ERR_OR_ZERO(bsi->ear_wake);
}

static void ear_enable_rst_isr()
{
    unsigned long irq_flags;
    spin_lock_irqsave(&rw_lock, irq_flags);
    if (test_bit(BT_RSTENABLE, &flags)) {
        spin_unlock_irqrestore(&rw_lock, irq_flags);
        return;
    }
    if (debug_mask & DEBUG_BTWAKE)
		BT_INFO("BT WAKE: enable the ear rst irq");
    set_bit(BT_RSTENABLE, &flags);
    spin_unlock_irqrestore(&rw_lock, irq_flags);
    if (enable_irq_wake(bsi->ear_rst_irq) < 0) {
        BT_ERR("Couldn't enable ear_rst as wakeup mode");
        return;
    }
    enable_irq(bsi->ear_rst_irq);
    return;
}

static void ear_disable_rst_isr()
{
    unsigned long irq_flags;
    spin_lock_irqsave(&rw_lock, irq_flags);
    if (!test_bit(BT_RSTENABLE, &flags)) {
        spin_unlock_irqrestore(&rw_lock, irq_flags);
        return;
    }
    if (debug_mask & DEBUG_BTWAKE)
		BT_INFO("BT WAKE: disable the ear rst irq");
	clear_bit(BT_RSTENABLE, &flags);
	spin_unlock_irqrestore(&rw_lock, irq_flags);
    if (disable_irq_wake(bsi->ear_rst_irq))
        BT_ERR("Couldn't disable ear_rst wakeup mode \n");
    disable_irq_nosync(bsi->ear_rst_irq);
    return;
}

static irqreturn_t ear_control_rst_isr(int irq, void *dev_id)
{
	//gpio_clear_detect_status(bsi->host_wake_irq);
	/* schedule a tasklet to handle the change in the host wake line */
	pm_stay_awake(bsi->dev);
	ear_disable_rst_isr();
	tasklet_schedule(&ear_rst_task);
	return IRQ_HANDLED;
}

static int ear_wake_host_isr_init()
{
    int ret;
	bsi->host_wake_irq = gpiod_to_irq(bsi->host_wake);
	if (bsi->host_wake_irq < 0) {
		BT_ERR("couldn't find host_wake irq\n");
		return -ENODEV;
	}

    BT_INFO("host_wake_irq %d", bsi->host_wake_irq);
    irq_set_status_flags(bsi->host_wake_irq, IRQ_NOAUTOEN);
    /* 上升沿唤醒 android, 下降沿允许 android 睡眠 */
    ret = request_irq(bsi->host_wake_irq, ear_control_hostwake_isr,
			            IRQF_TRIGGER_RISING,
			            "ear_hostwake", NULL);
	if (ret  < 0) {
		BT_ERR("Couldn't acquire BT_HOST_WAKE IRQ");
		return ret;
	}
    return ret;
}

static inline void ear_wake_host_isr_free()
{
    free_irq(bsi->host_wake_irq, NULL);
}

static int ear_rst_isr_init()
{
    int ret;
	bsi->ear_rst_irq = gpiod_to_irq(bsi->ear_rst);
	if (bsi->ear_rst_irq < 0) {
		BT_ERR("couldn't find ear rst irq\n");
		return -ENODEV;
	}

    BT_INFO("ear_rst_irq %d", bsi->ear_rst_irq);
    irq_set_status_flags(bsi->ear_rst_irq, IRQ_NOAUTOEN);
    ret = request_irq(bsi->ear_rst_irq, ear_control_rst_isr,
			            IRQF_TRIGGER_LOW,
			            "ear_rst", NULL);
	if (ret  < 0) {
		BT_ERR("Couldn't acquire EAR RST IRQ");
		return ret;
	}
    return ret;
}

static inline void ear_rst_isr_free()
{
    ear_disable_rst_isr();
    free_irq(bsi->ear_rst_irq, NULL);
}

static inline void ear_isr_free()
{
    ear_wake_host_isr_free();
    ear_rst_isr_free();
}

static int ear_isr_init()
{
    if(ear_wake_host_isr_init() < 0) {
        return -1;
    }
    if (ear_rst_isr_init() < 0) {
        goto free_host_irq;
    }
    return 0;

free_host_irq:
    ear_wake_host_isr_free();
    return -1;
}


static const struct genl_multicast_group ear_control_mcgrps[] = {
	{ .name = "events", },
};


static struct genl_family ear_control_family __ro_after_init = {
	.hdrsize        = 0,
	.name           = "EAR_CONTROL",
	.version        = 1,
	.maxattr	= 0,
	.module		= THIS_MODULE,
	.mcgrps		= ear_control_mcgrps,
	.n_mcgrps	= ARRAY_SIZE(ear_control_mcgrps),
};


static int ear_nl_int()
{
    return genl_register_family(&ear_control_family);
}

static void ear_nl_release()
{
    genl_unregister_family(&ear_control_family);
    return;
}



static int  ear_control_probe(struct platform_device *pdev)
{
	int ret;
	//struct resource *res;
    BT_INFO("enter the fun: %s", __func__);
	bsi = kzalloc(sizeof(struct bluesleep_info), GFP_KERNEL);
	if (!bsi)
		return -ENOMEM;
		
	bsi->dev = get_device(&pdev->dev);
    ret = ear_control_gpio_request(&pdev->dev);
    if (ret) {
        BT_ERR("request gpio failed\n");
        goto free_bsi;
    }
	ret = ear_isr_init();
    if (ret) {
        BT_ERR("init irq failed\n");
        goto free_bsi;
    }
    ret = device_init_wakeup(&pdev->dev, true);
    if (ret < 0) {
        BT_ERR("init the ear_control to a wakeup source failed\n");
        goto free_irqs;
    }
    
	return 0;
	
free_irqs:
    ear_isr_free();
free_bsi:
    put_device(bsi->dev);
	kfree(bsi);
	return ret;
}

static int ear_control_remove(struct platform_device *pdev)
{
	ear_control_ear_wake();
    if (test_bit(BT_WAKEUPHOST_ENABLE, &flags)) {
        ear_control_disable_irq(bsi->host_wake_irq, bsi->isFT);
	}
	del_timer(&tx_timer);
    ear_isr_free();
	device_init_wakeup(&pdev->dev, false);
	put_device(bsi->dev);
	kfree(bsi);
	return 0;
}

static struct of_device_id ear_control_match_table[] = {
	{ .compatible = "zeku,ear_control" },
	{}
};


static struct platform_driver ear_control_driver = {
	.probe = ear_control_probe,
	.remove = ear_control_remove,
	.driver = {
		.name = "ear_control",
		.owner = THIS_MODULE,
		.of_match_table = ear_control_match_table,
	},
};

static int ear_control_proc_show(struct seq_file *m, void *v)
{
	switch ((long)m->private) {
	case PROC_BTWAKE:
		seq_printf(m, "btwake:%u\n", !test_bit(BT_EXT_WAKE, &flags));
		break;
	case PROC_HOSTWAKE:
		seq_printf(m, "hostwake: %u\n", gpiod_get_value(bsi->host_wake));
		break;
	case PROC_WAKEUPHOSTENABLE:
		seq_printf(m, "ear_wakeup_host_enable: %u\n",
				test_bit(BT_WAKEUPHOST_ENABLE, &flags) ? 1 : 0);
		break;
	case PROC_ASLEEP:
		seq_printf(m, "asleep: %u\n",
				test_bit(BT_ASLEEP, &flags) ? 1 : 0);
	    break;
	case PROC_RSTENABLE:
		seq_printf(m, "irq enable: %u\n",
				test_bit(BT_RSTENABLE, &flags) ? 1 : 0);
		break;
	default:
		return 0;
	}
	return 0;
}


static int	ear_control_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ear_control_proc_show, PDE_DATA(inode));
}

static ssize_t	ear_control_proc_write(struct file *file, const char __user *buffer,
                                                    size_t count, loff_t *f_pos)
{
    void *data = PDE_DATA(file_inode(file));
	char lbuf[32];
	if (count >= sizeof(lbuf))
		count = sizeof(lbuf)-1;
	if (copy_from_user(lbuf, buffer, count))
		return -EFAULT;
	lbuf[count] = 0;
	
    switch ((unsigned long)data) {
    case PROC_BTWAKE:
		if (lbuf[0] == '1') {
			if (debug_mask & DEBUG_BTWAKE)
				BT_INFO("wakeup ear");
    		spin_lock(&rw_lock);
        	if (!test_bit(BT_EXT_WAKE, &flags)) {
        		spin_unlock(&rw_lock);
        		break;
        	}
			/* keep android wake */
			pm_stay_awake(bsi->dev);
			/* Pulling GPIO_8 high to wake the EAR */
			clear_bit(BT_ASLEEP, &flags);
			ear_control_ear_wake();
			spin_unlock(&rw_lock);
			/* start the timer */
            mod_timer(&tx_timer, (jiffies + (TX_TIMER_INTERVAL * HZ)));
		} else {
			if (debug_mask & DEBUG_BTWAKE)
				BT_INFO("allow ear to sleep");
			spin_lock(&rw_lock);
        	if (test_bit(BT_EXT_WAKE, &flags)) {
        		spin_unlock(&rw_lock);
        		break;
        	}
			/* Pulling GPIO_8 low allows the EAR to sleep */
			ear_control_ear_sleep();
			spin_unlock(&rw_lock);
		}
		break;
	case PROC_WAKEUPHOSTENABLE:
		if (lbuf[0] == '0')
			ear_control_disable_ear_wakeup_host(true); //stop ear wakeup host irq
		else if (lbuf[0] == '1')
			ear_control_enable_ear_wakeup_host(true); //start ear wakeup host irq
        else if (lbuf[0] == '2')
            ear_control_disable_ear_wakeup_host(false); //stop ear wakeup host irq
        else
            ear_control_enable_ear_wakeup_host(false); //start ear wakeup host irq
		break;
	case PROC_RSTENABLE:
	    if (lbuf[0] == '0') {
			ear_disable_rst_isr();
		} else {
			ear_enable_rst_isr();
		}	
		break;
	default:
		return 0;
    }
    return count;
}


static const struct proc_ops ear_control_proc_readwrite_fops = {
	.proc_open	= ear_control_proc_open,
	.proc_read   = seq_read,
	.proc_write  = ear_control_proc_write,
};
static const struct proc_ops ear_control_proc_read_fops = {
	.proc_open	= ear_control_proc_open,
	.proc_read   = seq_read,
};



/**
 * Initializes the module.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int  ear_control_init(void)
{
	int retval;
	struct proc_dir_entry *ent;

	BT_INFO("Ear control Driver Ver %s", VERSION);

	retval = platform_driver_register(&ear_control_driver);
	if (retval)
		return retval;

	bluetooth_dir = proc_mkdir("ear_control", NULL);
	if (bluetooth_dir == NULL) {
		BT_ERR("Unable to create /proc/ear_control directory");
		return -ENOMEM;
	}

	sleep_dir = proc_mkdir("sleep", bluetooth_dir);
	if (sleep_dir == NULL) {
		BT_ERR("Unable to create /proc/%s directory", PROC_DIR);
		return -ENOMEM;
	}
	/* Creating read/write "btwake" entry */
	ent = proc_create_data("btwake", S_IRUGO | S_IWUSR | S_IWGRP,
			sleep_dir, &ear_control_proc_readwrite_fops,
			(void *)PROC_BTWAKE);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/btwake entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}
	/* read only proc entries */
	ent = proc_create_data("hostwake", S_IRUGO, sleep_dir,
				&ear_control_proc_read_fops,
				(void *)PROC_HOSTWAKE);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/hostwake entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}
	/* read/write proc entries */
	ent = proc_create_data("wakeupenable", S_IRUGO | S_IWUSR | S_IWGRP,
			sleep_dir, &ear_control_proc_readwrite_fops,
			(void *)PROC_WAKEUPHOSTENABLE);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/proto entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}
	/* read only proc entries */
	ent = proc_create_data("asleep", S_IRUGO,
			sleep_dir, &ear_control_proc_read_fops,
			(void *)PROC_ASLEEP);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/asleep entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* read/write proc entries */

	ent = proc_create_data("rstenable", S_IRUGO | S_IWUSR | S_IWGRP,
			bluetooth_dir, &ear_control_proc_readwrite_fops,
			(void *)PROC_RSTENABLE);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/rstenable entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	flags = 0; /* clear all status bits */

	/* Initialize spinlock. */
	spin_lock_init(&rw_lock);

	/* Initialize timer */
#if (KERNEL_VERSION(4, 15, 0) > LINUX_VERSION_CODE)
	init_timer(&tx_timer);
	tx_timer.function = ear_control_tx_timer_expire;
	tx_timer.data = 0;
#else
	timer_setup(&tx_timer, ear_control_tx_timer_expire, 0);
#endif

	/* initialize host wake tasklet */
	tasklet_init(&hostwake_task,ear_control_hostwake_task, 0);
	tasklet_init(&ear_rst_task,ear_control_rst_task, 0);
	
    if (ear_nl_int()) {
        BT_ERR("init netlink failed");
        goto fail;
    }
	/* assert bt wake */
	//ear_control_ear_wake();
	BT_INFO("Ear control Driver int end");
	return 0;

fail:
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("wakeupenable", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("rstenable", bluetooth_dir);
	remove_proc_entry("ear_control", 0);
	return retval;
}

/**
 * Cleans up the module.
 */
static void  ear_control_exit(void)
{
	/* assert bt wake */

	platform_driver_unregister(&ear_control_driver);

	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("wakeupenable", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("rstenable", bluetooth_dir);
	remove_proc_entry("ear_control", 0);
	ear_nl_release();
}

module_init(ear_control_init);
module_exit(ear_control_exit);

MODULE_DESCRIPTION("Ear Control Driver ver %s " VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
