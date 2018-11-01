#############################################################
# Required variables for each makefile
# Discard this section from all parent makefiles
# Expected variables (with automatic defaults):
#   CSRCS (all "C" files in the dir)
#   SUBDIRS (all subdirs with a Makefile)
#   GEN_LIBS - list of libs to be generated ()
#   GEN_IMAGES - list of object file images to be generated ()
#   GEN_BINS - list of binaries to be generated ()
#   COMPONENTS_xxx - a list of libs/objs in the form
#     subdir/lib to be extracted and rolled up into
#     a generated lib/image xxx.a ()
#
TARGET = eagle
#FLAVOR = release
FLAVOR = debug

#EXTRA_CCFLAGS += -u
#parent_dir:=$(abspath $(shell pwd)/$(lastword $(MAKEFILE_LIST)))
#parent_dir:=$(shell dirname $(parent_dir))
#parent_dir:=$(shell dirname $(parent_dir))
#
#SDK_PATH= $(parent_dir)
#BIN_PATH=$(SDK_PATH)/bin

SDK_PATH= /home/aithinker/project/ESP8266_XSL_PRO
BIN_PATH=$(SDK_PATH)/bin

ifndef PDIR # {
GEN_IMAGES= eagle.app.v6.out
GEN_BINS= eagle.app.v6.bin
SPECIAL_MKTARGETS=$(APP_MKTARGETS)
#-----XSL-----info
# 添加编译的文件夹
#-------------
SUBDIRS=    \
	user    \
	driver  \
	xsl_bsp \
	xsl_config \
	xsl_module \
	xsl_ota
	
endif # } PDIR

LDDIR = $(SDK_PATH)/ld

CCFLAGS += -Os

TARGET_LDFLAGS =		\
	-nostdlib		\
	-Wl,-EL \
	--longcalls \
	--text-section-literals

ifeq ($(FLAVOR),debug)
    TARGET_LDFLAGS += -g -O2
endif

ifeq ($(FLAVOR),release)
    TARGET_LDFLAGS += -g -O0
endif
#-----XSL-----info
# 把编译好的库文件加到项目
#-------------
COMPONENTS_eagle.app.v6 = \
	user/libuser.a  \
	driver/libdriver.a  \
	xsl_bsp/libxsl_bsp.a \
	xsl_config/libxsl_config.a \
	xsl_module/libxsl_module.a \
	xsl_ota/libxsl_ota.a

#-----XSL-----info
#	加载第三方库文件
#	删除如下无用库
#		-ljson	\
#-------------
LINKFLAGS_eagle.app.v6 = \
	-L$(SDK_PATH)/lib        \
	-Wl,--gc-sections   \
	-nostdlib	\
    -T$(LD_FILE)   \
	-Wl,--no-check-sections	\
    -u call_user_start	\
	-Wl,-static						\
	-Wl,--start-group					\
	-lcirom \
	-lcrypto	\
	-lespconn	\
	-lespnow	\
	-lfreertos	\
	-lgcc					\
	-lhal					\
	-llwip	\
	-lmain	\
	-lmesh	\
	-lmirom	\
	-lmqtt	\
	-lnet80211	\
	-lnopoll	\
	-lopenssl               \
	-lphy	\
	-lpp	\
	-lpwm	\
	-lsmartconfig	\
	-lspiffs	\
	-lssl	\
	-lwpa	\
	-lwps		\
	$(DEP_LIBS_eagle.app.v6)					\
	-Wl,--end-group

DEPENDS_eagle.app.v6 = \
                $(LD_FILE) \
                $(LDDIR)/eagle.rom.addr.v6.ld

#############################################################
# Configuration i.e. compile options etc.
# Target specific stuff (defines etc.) goes in here!
# Generally values applying to a tree are captured in the
#   makefile at its root level - these are then overridden
#   for a subtree within the makefile rooted therein
#

#UNIVERSAL_TARGET_DEFINES =		\

# Other potential configuration flags include:
#	-DTXRX_TXBUF_DEBUG
#	-DTXRX_RXBUF_DEBUG
#	-DWLAN_CONFIG_CCX
#   -DMQTT_TASK: Define MQTT_TASK to enable MQTT start background
#                thread for a client.
CONFIGURATION_DEFINES =	-DICACHE_FLASH -DMQTT_TASK

DEFINES +=				\
	$(UNIVERSAL_TARGET_DEFINES)	\
	$(CONFIGURATION_DEFINES)

DDEFINES +=				\
	$(UNIVERSAL_TARGET_DEFINES)	\
	$(CONFIGURATION_DEFINES)

#############################################################

#############################################################
# Recursion Magic - Don't touch this!!
#
# Each subtree potentially has an include directory
#   corresponding to the common APIs applicable to modules
#   rooted at that subtree. Accordingly, the INCLUDE PATH
#   of a module can only contain the include directories up
#   its parent path, and not its siblings
#
# Required for each makefile to inherit from the parent
#
#-----XSL-----info
# 设置头文件路径
#-------------
INCLUDES := $(INCLUDES) -I $(PDIR)include
INCLUDES += -I $(PDIR)xsl_bsp
INCLUDES += -I $(PDIR)xsl_config 
INCLUDES += -I $(PDIR)xsl_module
INCLUDES += -I $(PDIR)xsl_ota
sinclude $(SDK_PATH)/Makefile

.PHONY: FORCE
FORCE:

