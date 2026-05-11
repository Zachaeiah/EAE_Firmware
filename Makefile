APP_NAME = main
BUILD_DIR = build
BUILD_TYPE = Debug
GENERATOR = MinGW Makefiles

.PHONY: all configure build run clean rebuild

all: build

configure:
	cmake -S . -B $(BUILD_DIR) -G "$(GENERATOR)" -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)

build: configure
	cmake --build $(BUILD_DIR)

run: build
	$(BUILD_DIR)/$(APP_NAME).exe

clean:
	cmake -E remove_directory $(BUILD_DIR)

rebuild: clean build