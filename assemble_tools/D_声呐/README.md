# Sonar相关的工具说明
 - sonar_config.yaml, 配合write_sonar_address.py使用
 - write_sonar_address, 根据sonar_config.yaml的配置，自动把地址写入sonar中，并测试
 - test_front_sonar.py，测试安装在前面的5个sonar
 - test_front_low_sonar.py，测试安装在前面较低位置的3个sonar
 - test_rear_sonar.py，测试安装在后面的3个sonar

# 写入地址的步骤
## 修改地址 
根据写入地址的需要,修改sonar_config.yaml文件中的old_addr和new_addr

## 执行脚本
```python
python write_sonar_address.py
