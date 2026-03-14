## camera/Makefile — Build pose_estimate_service độc lập
##
## Thứ tự biến: ARUCO_SO → ARUCO_VERSION → ARUCO_TAG → LDFLAGS → targets
##
## Sử dụng:
##   make deps         # tải aruco headers về vendor/include/ (chạy 1 lần)
##   make              # build (tự build-aruco nếu không có system aruco)
##   make build-aruco  # build aruco static lib từ source (robot không có contrib)
##   make clean        # xoá binary và static lib

CXX      := g++
CXXFLAGS := -Wall -Wextra -std=c++17 \
             -I../inc \
             -Ivendor/include

TARGET := pose_estimate_service
SRC    := pose_estimate.cpp

# ---------- OpenCV flags ----------
OPENCV_FLAGS := $(shell pkg-config --cflags --libs opencv4 2>/dev/null || \
                        pkg-config --cflags --libs opencv  2>/dev/null)
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null || \
                         pkg-config --cflags opencv 2>/dev/null)

# ============================================================
# Bước 1: Detect ARUCO_SO (loại /snap — snap libs không tương thích GLIBC)
# ============================================================
ARUCO_SO := $(shell ldconfig -p 2>/dev/null \
                | grep 'libopencv_aruco\.so\.' \
                | awk '{print $$NF}' \
                | grep -v '^/snap' | tail -1)
ifeq ($(ARUCO_SO),)
    ARUCO_SO := $(shell find /usr /usr/local /opt -name 'libopencv_aruco.so.*' \
                    -not -type l -not -path '*/snap/*' 2>/dev/null | sort | tail -1)
endif

# ============================================================
# Bước 2: Tính ARUCO_VERSION và ARUCO_TAG
#   so.4.2   → version=4.2   → tag=4.2.0
#   so.4.5.4 → version=4.5.4 → tag=4.5.4
#   (không)  → pkg-config    → fallback 4.5.4
# ============================================================
ARUCO_VERSION := $(patsubst libopencv_aruco.so.%,%,$(notdir $(ARUCO_SO)))
ifeq ($(ARUCO_VERSION),)
    ARUCO_VERSION := $(shell pkg-config --modversion opencv4 2>/dev/null || \
                             pkg-config --modversion opencv  2>/dev/null || \
                             echo "4.5.4")
endif
ARUCO_TAG := $(shell echo "$(ARUCO_VERSION)" | \
               awk -F. 'NF==2{print $$0".0"} NF>=3{print $$0}')
ifeq ($(ARUCO_TAG),)
    ARUCO_TAG := 4.5.4
endif

ARUCO_CONTRIB_BASE := https://raw.githubusercontent.com/opencv/opencv_contrib/$(ARUCO_TAG)/modules/aruco

# ============================================================
# Bước 3: Set LDFLAGS
# ============================================================
ifneq ($(ARUCO_SO),)
    ARUCO_LDFLAG := $(ARUCO_SO)
    ARUCO_RPATH  := -Wl,-rpath,$(dir $(ARUCO_SO))
    $(info [camera] Linking aruco: $(ARUCO_SO))
else
    ARUCO_LDFLAG := vendor/aruco/libaruco_contrib.a
    ARUCO_RPATH  :=
    $(info [camera] No system aruco → will build local static lib (tag: $(ARUCO_TAG)))
endif

LDFLAGS := $(ARUCO_LDFLAG) $(OPENCV_FLAGS) $(ARUCO_RPATH)

# ============================================================
# Targets
# ============================================================
.PHONY: all clean deps build-aruco

ifeq ($(ARUCO_SO),)
all: vendor/include/opencv2/aruco.hpp vendor/aruco/libaruco_contrib.a $(TARGET)
else
all: vendor/include/opencv2/aruco.hpp $(TARGET)
endif

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)
	@echo "[camera] Build done: ./$(TARGET)"

# ============================================================
# deps: tải aruco headers về vendor/include/
# ============================================================
deps: vendor/include/opencv2/aruco.hpp

vendor/include/opencv2/aruco.hpp:
	@echo "[deps] opencv: $(ARUCO_VERSION)  tag: $(ARUCO_TAG)"
	@mkdir -p vendor/include/opencv2/aruco
	wget -q -O vendor/include/opencv2/aruco.hpp \
	    $(ARUCO_CONTRIB_BASE)/include/opencv2/aruco.hpp || \
	    { echo "[deps] ERROR: download failed for tag $(ARUCO_TAG)"; exit 1; }
	wget -q -O vendor/include/opencv2/aruco/dictionary.hpp \
	    $(ARUCO_CONTRIB_BASE)/include/opencv2/aruco/dictionary.hpp  || true
	wget -q -O vendor/include/opencv2/aruco/aruco_calib.hpp \
	    $(ARUCO_CONTRIB_BASE)/include/opencv2/aruco/aruco_calib.hpp || true
	wget -q -O vendor/include/opencv2/aruco/charuco.hpp \
	    $(ARUCO_CONTRIB_BASE)/include/opencv2/aruco/charuco.hpp     || true
	@echo "[deps] Done → vendor/include/"

# ============================================================
# build-aruco: compile aruco module từ opencv_contrib source
#   Dùng khi không có system aruco (robot2: OpenCV 4.5 không có contrib)
# ============================================================
build-aruco: vendor/aruco/libaruco_contrib.a

vendor/aruco/libaruco_contrib.a: vendor/aruco/src/aruco.cpp vendor/include/opencv2/aruco.hpp
	@echo "[build-aruco] Compiling (tag $(ARUCO_TAG))..."
	@mkdir -p vendor/aruco/obj
	cd vendor/aruco/obj && $(CXX) -std=c++17 -O2 -fPIC \
	    -I../../include \
	    -I../src \
	    $(OPENCV_CFLAGS) \
	    -c ../src/*.cpp
	ar rcs $@ vendor/aruco/obj/*.o
	@echo "[build-aruco] Done: $@"

vendor/aruco/src/aruco.cpp:
	@echo "[build-aruco] Downloading ALL src files (tag $(ARUCO_TAG))..."
	@mkdir -p vendor/aruco/src
	@for f in \
	    aruco.cpp \
	    apriltag_quad_thresh.cpp \
	    charuco.cpp \
	    dictionary.cpp \
	    zmaxheap.cpp \
	    apriltag_quad_thresh.hpp \
	    precomp.hpp \
	    predefined_dictionaries.hpp \
	    predefined_dictionaries_apriltag.hpp \
	    unionfind.hpp \
	    zarray.hpp \
	    zmaxheap.hpp \
	; do \
	    wget -q -O vendor/aruco/src/$$f $(ARUCO_CONTRIB_BASE)/src/$$f || \
	        echo "[build-aruco] WARNING: $$f not found (skip)"; \
	done
	@echo "[build-aruco] Sources ready."

clean:
	rm -f $(TARGET)
	rm -rf vendor/aruco/obj vendor/aruco/libaruco_contrib.a vendor/aruco/src
