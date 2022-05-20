echo -e "\e[31m**** 注意 ****\e[m"
echo -e "\e[31mStretch関連のノードを落としてください\e[m"
echo -e "\e[31mバッテリーの電圧を確認するためアダプターは外して実行してください\e[m"
echo -e "\e[31m***************\e[m"

pkill -f stretch_xbox*
stretch_robot_battery_check.py
read -p "Hit enter to quit. "