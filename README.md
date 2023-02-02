# PICO Linetracer DC

## 리눅스 환경에서 개발 환경 구축
Ubuntu 20.04 환경에서 Raspberry Pi Pico 개발 환경을 구축할 수 있다.

1. 빌드 필수 패키지를 설치한다.
    
    ```bash
    sudo apt update
    sudo apt install git wget cmake build-essential
    ```
    
1. ARM 크로스 컴파일러를 설치한다.
    - [Arm GNU Toolchain Downloads](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)에서 자신에게 맞는 운영체제의 Toolchain를 다운받는다. 보통 x86 시스템의 리눅스 환경을 많이 사용하며, 아래의 명령어로 다운로드 받을 수 있다.
    
    ```bash
    wget https://developer.arm.com/-/media/Files/downloads/gnu/12.2.rel1/binrel/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi.tar.xz
    ```
    
    - 다운로드 받은 압축 파일을 `/opt/arm-gnu-toolchain` 디렉터리에 푼다.
    
    ```bash
    sudo mkdir /opt/arm-gnu-toolchain
    sudo tar xvf arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi.tar.xz -C /opt/arm-gnu-toolchain --strip-components=1
    ```
    
    - 크로스 컴파일러에 접근할 수 있도록 환경 변수를 설정한다.
    
    ```bash
    echo "export PATH=\$PATH:/opt/arm-gnu-toolchain/bin" >> ~/.bashrc
    source ~/.profile
    ```
    
    - 아래와 같이 `arm-none-eabi-gcc --version` 명령어를 수행해서 버전 정보가 잘 나온다면 설치가 된 것이며, 설치가 끝났으므로 `rm` 명령어를 이용해서 Toolchain 압축 파일을 삭제한다.
    
    ```
    $ arm-none-eabi-gcc --version
    arm-none-eabi-gcc (Arm GNU Toolchain 12.2.Rel1 (Build arm-12.24)) 12.2.1 20221205
    Copyright (C) 2022 Free Software Foundation, Inc.
    This is free software; see the source for copying conditions.  There is NO
    warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    
    $ rm arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi.tar.xz
    ```
    
1. 프로젝트 디렉터리 안에서 pico-sdk를 내려받는다. 그리고 pico-sdk 디렉터리로 들어가 submodule을 다운로드 받는다.
    
    ```bash
    git clone https://github.com/raspberrypi/pico-sdk
    cd pico-sdk
    git submodule update --init
    cd ..
    ```
    
1. build 디렉터리를 만들어서 cmake를 수행한다.

    ```bash
    mkdir build
    cd build
    cmake ..
    ```
    
## 개발중 논의점

1. D term에 Low pass filter이 필요한가?


2. DC motor 제어에 D term이 필요한가?


3. PID 제어에는 Dead zone이 필요 없다.
