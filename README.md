# 듀얼 엘리베이터 제어 시스템 (Dual Elevator Control)

STM32F103RB 마이크로컨트롤러를 이용한 듀얼 엘리베이터 제어 펌웨어.

## 프로젝트 구조
- `Elevator_103/` : STM32 펌웨어 소스코드 및 프로젝트 파일 (STM32CubeIDE)
- `2021440073_안한영.pdf` : 프로젝트 기획서 및 보고서

## 하드웨어 및 개발 환경
- MCU : STM32F103RBT6
- Firmware : STM32CubeIDE, C (HAL API)

## 빌드 및 실행
1. STM32CubeIDE에서 `Elevator_103/` 프로젝트 임포트.
2. 빌드(Build) 후 ST-LINK를 통해 보드에 바이너리 다운로드.
