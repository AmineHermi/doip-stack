This project consists of building a a DoIP protocol stack simulation.
where i simulated LM3S6965 ,ARM Cortex-M3 via QEMU and implemented FreeRTOS.
I created 3 tasks (ethernetTask, arpTask, ipTask) that are interconnected through data flowing between layers with data originating in ethernet task( Role:filling the header) then transmitted up to the next layer which is the arp task(Role:validating the protocol version) that it ll handle the received DoIP packet and validate it s protocol version then resends it to the final layer, Ip task (Role: showing payload) which reveives the packet and renders the payload .

Build & Run:
git clone https://github.com/FreeRTOS/FreeRTOS.git --recurse-submodules --depth 1
cd FreeRTOS/FreeRTOS/Demo/CORTEX_LM3S6965_GCC_QEMU
cmake -B build -DCMAKE_TOOLCHAIN_FILE=gcc-arm-embedded.cmake
cmake --build build
qemu-system-arm -machine lm3s6965evb -nographic -kernel build/RTOSDemo.elf
