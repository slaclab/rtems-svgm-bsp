
#define FLAG_MAND	1
#define FLAG_NOUSE	2	/* dont put into the commandline at all */
#define FLAG_CLRBP  4	/* field needs to be cleared for bootp  */

typedef struct ParmRec_ {
	char	*name;
	char	**pval;
	int		flags;
} ParmRec, *Parm;


static char *boot_srvname=0;
static char *boot_use_bootp=0;
static char	*boot_my_ip=0;
static char	*boot_my_netmask=0;

static ParmRec parmList[]={
	{ "BP_FILE=",  &__BSP_dummy_bsdnet_bootp_file_name,
			FLAG_MAND,
	},
	{ "BP_SRVR=",  &boot_srvname,
			FLAG_MAND,
	},
	{ "BP_GTWY=",  &rtems_bsdnet_config.gateway,
			FLAG_CLRBP, 
	},
	{ "BP_MYIP=",  &boot_my_ip,
			FLAG_MAND | FLAG_CLRBP,
	},
	{ "BP_MYMK=",  &boot_my_netmask,
			FLAG_MAND | FLAG_CLRBP,
	},
	{ "BP_MYNM=",  &rtems_bsdnet_config.hostname,
			FLAG_CLRBP,
	},
	{ "BP_MYDN=",  &rtems_bsdnet_config.domainname,
			FLAG_CLRBP,
	},
	{ "BP_LOGH=",  &rtems_bsdnet_config.log_host,
			FLAG_CLRBP,
	},
	{ "BP_DNS1=",  &rtems_bsdnet_config.name_server[0],
			FLAG_CLRBP,
	},
	{ "BP_DNS2=",  &rtems_bsdnet_config.name_server[1],
			FLAG_CLRBP,
	},
	{ "BP_DNS3=",  &rtems_bsdnet_config.name_server[2],
			FLAG_CLRBP,
	},
	{ "BP_NTP1=",  &rtems_bsdnet_config.ntp_server[0],
			FLAG_CLRBP,
	},
	{ "BP_NTP2=",  &rtems_bsdnet_config.ntp_server[1],
			FLAG_CLRBP,
	},
	{ "BP_NTP3=",  &rtems_bsdnet_config.ntp_server[2],
			FLAG_CLRBP,
	},
	{ "BP_ENBL=",  &boot_use_bootp,
			0,
	},
	{ 0, }
};
