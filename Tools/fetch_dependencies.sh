#!/bin/sh
# Dependencias oficiais, mantidas fora do Git em Drivers/.
set -eu
cd "$(dirname "$0")/.."

fetch() {
    repo=$1 tag=$2 destination=$3 expected=$4
    if [ ! -d "$destination/.git" ]; then
        git clone --depth 1 --branch "$tag" "$repo" "$destination"
    fi
    actual=$(git -C "$destination" rev-parse HEAD)
    if [ "$actual" != "$expected" ]; then
        printf 'Versao inesperada em %s: %s\n' "$destination" "$actual" >&2
        exit 1
    fi
}

fetch https://github.com/STMicroelectronics/stm32f4xx-hal-driver.git v1.8.3 \
    Drivers/STM32F4xx_HAL_Driver c2e1406d7ea4b73aa42b98ddeb75a8670b1a5a16
fetch https://github.com/STMicroelectronics/cmsis-device-f4.git v2.6.10 \
    Drivers/CMSIS/Device/ST/STM32F4xx 5f41fb29d22773896c780052bf61e47fc924d524

if [ ! -d Drivers/CMSIS_Core/.git ]; then
    git clone --depth 1 --filter=blob:none --sparse --branch 5.9.0 \
        https://github.com/ARM-software/CMSIS_5.git Drivers/CMSIS_Core
fi
fetch https://github.com/ARM-software/CMSIS_5.git 5.9.0 \
    Drivers/CMSIS_Core 2b7495b8535bdcb306dac29b9ded4cfb679d7e5c
git -C Drivers/CMSIS_Core sparse-checkout set CMSIS/Core/Include

CUBE_REPO="${STM32CUBE_REPO:-$HOME/STM32Cube/Repository}"
mkdir -p Drivers/CMSIS/Device/ST

if [ -d "$CUBE_REPO/STM32Cube_FW_F7_V1.17.4" ]; then
    if [ ! -e Drivers/STM32F7xx_HAL_Driver ]; then
        ln -snf "$CUBE_REPO/STM32Cube_FW_F7_V1.17.4/Drivers/STM32F7xx_HAL_Driver" Drivers/STM32F7xx_HAL_Driver
    fi
    if [ ! -e Drivers/CMSIS/Device/ST/STM32F7xx ]; then
        ln -snf "$CUBE_REPO/STM32Cube_FW_F7_V1.17.4/Drivers/CMSIS/Device/ST/STM32F7xx" Drivers/CMSIS/Device/ST/STM32F7xx
    fi
fi

if [ -d "$CUBE_REPO/STM32Cube_FW_H7_V1.13.0" ]; then
    if [ ! -e Drivers/STM32H7xx_HAL_Driver ]; then
        ln -snf "$CUBE_REPO/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver" Drivers/STM32H7xx_HAL_Driver
    fi
    if [ ! -e Drivers/CMSIS/Device/ST/STM32H7xx ]; then
        ln -snf "$CUBE_REPO/STM32Cube_FW_H7_V1.13.0/Drivers/CMSIS/Device/ST/STM32H7xx" Drivers/CMSIS/Device/ST/STM32H7xx
    fi
fi
