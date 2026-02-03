#!/bin/bash

# Define output directory and file
OUTPUT_DIR="/home/pi/Desktop/Repo/ExosqueletonForWalkingAid/Analisis_de_Configuracion"
OUTPUT_FILE="$OUTPUT_DIR/rpi_config_report.txt"

# Create directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

echo "=== RASPBERRY PI CONFIGURATION REPORT ===" > "$OUTPUT_FILE"
echo "Generated: $(date)" >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

echo "=== HOSTNAME === [/etc/hostname]" >> "$OUTPUT_FILE"
cat /etc/hostname >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

echo "=== I2C CONFIGURATION ===" >> "$OUTPUT_FILE"
echo "I2C Status (raspi-config): $(sudo raspi-config nonint get_i2c)" >> "$OUTPUT_FILE"
echo "I2C Devices [/dev/i2c-*]:" >> "$OUTPUT_FILE"
ls /dev/i2c-* >> "$OUTPUT_FILE" 2>&1
echo "" >> "$OUTPUT_FILE"

echo "=== BOOT CONFIG (I2C related) === [/boot/config.txt]" >> "$OUTPUT_FILE"
cat /boot/config.txt | grep -i i2c >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

echo "=== LOADED MODULES === [/etc/modules]" >> "$OUTPUT_FILE"
cat /etc/modules >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

echo "=== PYTHON PACKAGES (Virtual Environment) === [/home/pi/Desktop/Repo/ExosqueletonForWalkingAid/tfm_env]" >> "$OUTPUT_FILE"
source /home/pi/Desktop/Repo/ExosqueletonForWalkingAid/tfm_env/bin/activate
pip3 list >> "$OUTPUT_FILE"
deactivate
echo "" >> "$OUTPUT_FILE"

echo "=== USER GROUPS === [/etc/group]" >> "$OUTPUT_FILE"
groups pi >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

echo "Report saved to $OUTPUT_FILE"
cat "$OUTPUT_FILE"
