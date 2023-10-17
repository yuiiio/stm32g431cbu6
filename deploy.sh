ELF_PATH="$1"

openocd -f openocd.cfg -c "program ${ELF_PATH} verify reset exit"
