# ============================================================================
# Makefile cho Single Mobile Robot - Incremental Build
# Sử dụng: make                  (build mặc định cho robot1)
#          make ROBOT_ID=robot2   (build cho robot2)
#          make clean             (xóa build)
#          make rebuild           (clean + build lại)
# ============================================================================

# --- Cấu hình Robot ID ---
ROBOT_ID ?= robot1

# Kiểm tra robot ID hợp lệ
ifneq ($(ROBOT_ID),robot1)
ifneq ($(ROBOT_ID),robot2)
$(error Invalid ROBOT_ID='$(ROBOT_ID)'. Use 'robot1' or 'robot2')
endif
endif

# Trích xuất số robot: robot1 -> 1, robot2 -> 2
ROBOT_NUM := $(subst robot,,$(ROBOT_ID))

# --- Compiler & Flags ---
CC       := gcc
CFLAGS   := -Wall -Wextra -std=c11 \
            -D_GNU_SOURCE -D_POSIX_C_SOURCE=200809L \
            -DROBOT_ID=$(ROBOT_NUM)
INCLUDES := -I./inc -I./vl53l0x
LDFLAGS  := -lsqlite3 -lpthread -lm -lgpiod -li2c

# --- Thư mục ---
BUILD_DIR := build/$(ROBOT_ID)
TARGET    := single_mini-server

# --- Source files ---
SRC_FILES     := $(wildcard src/*.c)
VL53_FILES    := $(filter-out vl53l0x/vl53l0x_addr_init.c, $(wildcard vl53l0x/*.c))
DB_FILES      := $(wildcard database/*.c)
ALL_SOURCES   := $(SRC_FILES) $(VL53_FILES) $(DB_FILES)

# --- Object files (đặt trong build/ để không làm bẩn thư mục source) ---
# src/main.c       -> build/robot1/src/main.o
# vl53l0x/xxx.c    -> build/robot1/vl53l0x/xxx.o
# database/xxx.c   -> build/robot1/database/xxx.o
ALL_OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(ALL_SOURCES))

# --- Header files (dùng cho dependency tracking) ---
INC_HEADERS  := $(wildcard inc/*.h)
VL53_HEADERS := $(wildcard vl53l0x/*.h)

# ============================================================================
# RULES
# ============================================================================

.PHONY: all clean rebuild info

all: $(TARGET)
	@echo ""
	@echo "✅ Build thành công cho $(ROBOT_ID)! Chạy với ./$(TARGET)"

# Link tất cả object files thành binary
$(TARGET): $(ALL_OBJECTS)
	@echo "[LINK] $@"
	@$(CC) $(ALL_OBJECTS) -o $@ $(LDFLAGS)

# --- Compile rules với auto-dependency generation ---
# GCC tự tạo file .d chứa dependency của mỗi .c file
# Khi bạn sửa 1 header, chỉ các .c file include header đó mới được build lại

# Rule cho src/*.c
$(BUILD_DIR)/src/%.o: src/%.c
	@mkdir -p $(dir $@)
	@echo "[CC]   $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -MMD -MP -c $< -o $@

# Rule cho vl53l0x/*.c
$(BUILD_DIR)/vl53l0x/%.o: vl53l0x/%.c
	@mkdir -p $(dir $@)
	@echo "[CC]   $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -MMD -MP -c $< -o $@

# Rule cho database/*.c
$(BUILD_DIR)/database/%.o: database/%.c
	@mkdir -p $(dir $@)
	@echo "[CC]   $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -MMD -MP -c $< -o $@

# Include auto-generated dependency files (nếu tồn tại)
# File .d được tạo bởi -MMD -MP, chứa danh sách header mà mỗi .c file phụ thuộc
-include $(ALL_OBJECTS:.o=.d)

# --- Clean ---
clean:
	@echo "🧹 Cleaning build files..."
	@rm -rf build/
	@rm -f $(TARGET)
	@echo "Done."

# --- Rebuild (clean + build) ---
rebuild: clean all

# --- Thông tin build ---
info:
	@echo "ROBOT_ID  : $(ROBOT_ID)"
	@echo "ROBOT_NUM : $(ROBOT_NUM)"
	@echo "CC        : $(CC)"
	@echo "CFLAGS    : $(CFLAGS)"
	@echo "INCLUDES  : $(INCLUDES)"
	@echo "LDFLAGS   : $(LDFLAGS)"
	@echo "BUILD_DIR : $(BUILD_DIR)"
	@echo "TARGET    : $(TARGET)"
	@echo ""
	@echo "Sources ($(words $(ALL_SOURCES)) files):"
	@$(foreach s,$(ALL_SOURCES),echo "  $(s)";)
	@echo ""
	@echo "Objects ($(words $(ALL_OBJECTS)) files):"
	@$(foreach o,$(ALL_OBJECTS),echo "  $(o)";)
