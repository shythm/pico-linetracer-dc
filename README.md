# PICO Linetracer DC

## 개발 환경 구축

1. ARM 크로스 컴파일러 다운로드

[ARM 공식 홈페이지](https://developer.arm.com/downloads/-/gnu-rm)에서 자신에게 맞는 운영체제의 컴파일러를 다운받는다. 보통 x86 시스템의 리눅스 환경을 많이 사용하므로, 아래의 명령어를 통해 다운로드할 수 있다.

```sh
$ wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
```

2. ARM 크로스 컴파일러 설치

다운로드한 컴파일러의 압축을 풀고, 터미널에서 해당 바이너리로 접근할 수 있도록 환경변수를 설정해준다. 이때 pico에서 사용하는 컴파일러 경로 환경변수 이름은 `PICO_TOOLCHAIN_PATH`이다. 참고로 아래 명령어 수행 후 터미널을 재시작 해줘야 환경변수가 제대로 적용된다.

```sh
$ sudo tar -xvf gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 -C /usr/share

$ echo "export PICO_TOOLCHAIN_PATH=/usr/share/gcc-arm-none-eabi-10.3-2021.10" >> ~/.bashrc
```

3. pico-sdk 내려받기

```sh
$ git clone https://github.com/raspberrypi/pico-sdk
```

4. 필요한 바이너리 설치

```sh
$ sudo apt install build-essential cmake
```

5. build 폴더 만들고, CMake 수행

```sh
$ mkdir build
$ cd build
$ cmake ..
```
