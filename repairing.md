# バッテリーのリカバリーに関するメモ

参考サイト：https://docs.hello-robot.com/0.1/battery_maintenance_guide/#repairing-damaged-batteries

## リカバリーについて
- Stretchのバッテリーが損傷し，ロボットの充電を維持するのが難しい場合に行う

## 充電器のインターフェース  
  <img src="figs/battery.png" alt="switch" width="500">

## 手順
1. Stretchの電源スイッチを切り，充電器をロボットから取り外す
1. 充電器を12V SUPPLYモードにする  
  <img src="figs/battery_supply.png" alt="switch" width="500">
1. 充電器を取り付け，Stretchを4-8時間充電する  

1. 充電器を12V REPAIRモードにする  
  <img src="figs/battery_repair.png" alt="switch" width="500">
1. 充電がスタンバイに戻るまで充電する  
  <img src="figs/battery_standby.png" alt="switch" width="500">  
  ※公式サイトには最大4時間とあるが，4時間を超える場合もある
1. 充電器を12V AGMモードに変更して終了  
  <img src="figs/battery_agm.png" alt="switch" width="500">