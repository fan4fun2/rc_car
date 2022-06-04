#!/usr/bin/env python
# -*- coding: utf-8 -*-

#==============================================================================
# ラジコン・カーをテストするＧＵＩとＲＯＳを橋渡しするブリッジ・プログラム
#                                                       RosBrdgGui.py Ver.3.1
#                                                       fan4fun2rc 29 Apr. 2022
#------------------------------------------------------------------------------
#　V1:ラジコン・カーの速度情報とパラメータをTkinterのGUIを使用して視覚的にテスト
#　   できるようにしたGuiRcTest.pyとRaspiで実行するROSの制御プログラムを接続する
#　   ブリッジ・プログラム。
#　V2:目標速度制御を行うときのＰＷＭにオフセットを追加したため、ＧＵＩも変更となり
#　   プロトコルにも互換がないためメジャー・バージョンの変更とした。
#　V3:ラズパイ本体で電圧監視するプログラムRosRaspiCheckPower.pyとＧＵＩ連携する
#　   ためにトピック/raspi_powerと/raspi_shellを橋渡しする。
#
#【仕様】
#　・GUIは別ファイルとしてGuiRcTest.pyとしてインポートする
#　・速度情報はRosSpCon.pyからの/speed_infoを購読してGUIに表示する
#　・GUIで設定された目標速度、または、PWMはRosSpCon.pyに/drive_infoとして配信する
#　・GUIで設定された速度係数、比例係数、積分係数、微分係数はRosSpCon.pyに
#　　/param_infoとして配信する
#　・電圧監視情報はRosRaspiCheckPower.pyからの/raspi_powerを購読してGUIに表示する
#　・GUIで指示されたシャットダウン・コマンドを/raspi_shellとして配信する
#
#【関数概要】
#　１．GUIで駆動情報が変更されたときにコールバックされる関数：driveCallback(s, i)
#　２．GUIで制御係数が変更されたときにコールバックされる関数：paramCallback(s,l,p,i,d)
#　３．ROSの速度トピックを受信したときにコールバックされる関数：speedCallback(roi)
#　４．GUIでシャットダウンが指示されたときにコールバックされる関数：shellCallback(s)
#　５．ROSの電圧監視トピックを受信したときにコールバックされる関数：powerCallback(f)
#　６．メイン処理関数：main()
#
#【リビジョン・ヒストリ】
#　・V1.0-2022/03/12:コーディング開始
#　・V2.0-2022/03/14:目標速度制御を行うときのＰＷＭにオフセットを追加
#　・V3.0-2022/03/16:RosRaspiCheckPower.pyとのトピック通信処理を追加
#　・V3.1-2022/04/29:ノード名が分かりにくいので「rc_car」から「rc_car_pc」に変更
#==============================================================================

#------------------------------------------------------------------------------
#　インポートするライブラリの宣言
#------------------------------------------------------------------------------
import rospy
from sensor_msgs.msg import RegionOfInterest as ROI
from std_msgs.msg import Float32
from std_msgs.msg import String
import GuiRcTestV3 as rc


#==============================================================================
# １．GUIで駆動情報が変更されたときにコールバックされる関数：driveCallback(s, i)
#   ＜引数  s：左/右と目標速度/ＰＷＭを区別する文字列で'LT/LW/RT/RW'の４種類＞
#   ＜引数  i：目標速度[mm/s]またはＰＷＭ[%]の比率を表す整数で正は前進、負は後退＞
#------------------------------------------------------------------------------
#　・sensor_msgs/RegionOfInterest.msgを使用して（独自型を作成せず）
#　　‐bool do_rectify ：左ならTrue、右ならFalseにエンコード
#　　‐uint32 width    ：目標速度なら１、ＰＷＭなら０にエンコード
#　　‐uint32 height   ：目標速度またはＰＷＭの整数をエンコード
#　　‐uint32 y_offset ：前進方向なら０、後退方向なら１にエンコード
#==============================================================================
def driveCallback(s, i):
    drv = ROI()
    drv.do_rectify = True if s == 'LT' or s == 'LW' else False
    drv.width = 1 if s == 'LT' or s == 'RT' else 0
    #
    # 利用している変数[height]が符号なしなので符号を[y_offset]に設定
    #
    if i < 0:
        drv.height = -i
        drv.y_offset = 1
    else:
        drv.height = i
        drv.y_offset = 0
    driveCallback.pub.publish(drv)


#==============================================================================
# ２．GUIで制御係数が変更されたときにコールバックされる関数：paramCallback(s,l,p,i,d,o)
#   ＜引数  s：左/右を区別する文字列で'L/R'の２種類のみ＞
#   ＜引数  l：目標速度に応じたモーターへの駆動係数＞
#   ＜引数  p：ＰＩＤ制御の偏差に対する比例制御部の係数＞
#   ＜引数  i：ＰＩＤ制御の偏差に対する積分制御部の係数＞
#   ＜引数  d：ＰＩＤ制御の偏差に対する微分制御部の係数＞
#   ＜引数  o：速度制御時のモーターへのＰＷＭオフセット＞
#------------------------------------------------------------------------------
#　・sensor_msgs/RegionOfInterest.msgを使用して（独自型を作成せず）
#　　‐bool do_rectify ：左ならTrue、右ならFalseにエンコード
#　　‐uint32 width    ：目標速度に応じたモーターへの駆動係数
#　　‐uint32 height   ：ＰＩＤ制御の偏差に対する比例制御部の係数
#　　‐uint32 y_offset ：ＰＩＤ制御の偏差に対する積分制御部の係数
#　　‐uint32 x_offset ：ＰＩＤ制御の偏差に対する微分制御部の係数＋ＰＷＭオフセット
#==============================================================================
def paramCallback(s, l, p, i, d, o):
    prm = ROI()
    prm.do_rectify = True if s == 'L' else False
    prm.width = l if l >= 0 else -l     # 負の値があれば絶対値に修正
    prm.height = p if p >= 0 else -p
    prm.y_offset = i if i >= 0 else -i
    dd = d if d >= 0 else -d
    dd %= 256                           # 8ビット256以下に丸める
    oo = o if o >= 0 else -o
    prm.x_offset = dd + oo * 256        # 上位にオフセット下位に微分係数をパック
    paramCallback.pub.publish(prm)


#==============================================================================
# ３．ROSの速度トピックを受信したときにコールバックされる関数：speedCallback(roi)
#   ＜引数 roi：左/右の車輪の速度を下記のようにエンコードしている＞
#------------------------------------------------------------------------------
#　・sensor_msgs/RegionOfInterest.msgを使用して（独自型を作成せず）
#　　‐bool do_rectify ：未使用、常にFalse
#　　‐uint32 width    ：左車輪の速度[mm/s]の絶対値
#　　‐uint32 height   ：右車輪の速度[mm/s]の絶対値
#　　‐uint32 y_offset ：左車輪が前進方向なら０、後退方向なら１にエンコード
#　　‐uint32 x_offset ：右車輪が前進方向なら０、後退方向なら１にエンコード
#==============================================================================
def speedCallback(roi):
    lsp = roi.width  if roi.y_offset == 0 else -roi.width
    rsp = roi.height if roi.x_offset == 0 else -roi.height
    rc.setSpeed([int(lsp),int(rsp)])


#==============================================================================
# ４．GUIでシャットダウンが指示されたときにコールバックされる関数：shellCallback(s)
#   ＜引数  s：シェルに発行されるコマンド文字列＞
#------------------------------------------------------------------------------
#　・受信した文字列をトピック/raspi_shellに配信する
#==============================================================================
def shellCallback(s):
    cmd = String()
    cmd.data = s
    shellCallback.pub.publish(cmd)


#==============================================================================
# ５．ROSの電圧監視トピックを受信したときにコールバックされる関数：powerCallback(f)
#   ＜引数 f：受信したトピックのメッセージFloat32型の電圧値＞
#------------------------------------------------------------------------------
#　・受信したトピック/raspi_powerからの電圧値をＧＵＩに送信する
#==============================================================================
def powerCallback(f):
    rc.setPower(f.data)


#==============================================================================
# ６．メイン処理関数：main()
#------------------------------------------------------------------------------
#　・ROSのノードを[rc_car_pc]の名称で作成
#　・ＲＯＳのトピック[/speed_info]を受信した時にコールバックする関数を設定
#　・ＲＯＳのトピックを[/drive_info]と[/param_info]の名称で作成
#　・駆動情報と制御係数が変更されたときに通知するコールバック関数を設定
#　・ＲＯＳのトピック[/raspi_power]を受信した時にコールバックする関数を設定
#　・シャットダウンが指示されたときに通知するコールバック関数を設定
#　・メインループはＧＵＩで終了まで実行
#==============================================================================
def main():
    rospy.init_node('rc_car_pc')
    rospy.loginfo("Start RosBrdgGui.py with GuiRcTest.py...")
    rospy.Subscriber("/speed_info", ROI, speedCallback, queue_size=10)
    driveCallback.pub = rospy.Publisher('/drive_info', ROI, queue_size=10)
    paramCallback.pub = rospy.Publisher('/param_info', ROI, queue_size=10)
    #
    # 駆動情報と制御係数が変更されたときに通知するコールバック関数を設定する
    #
    rc.setCallbackDrive(driveCallback)
    rc.setCallbackParam(paramCallback)
    #
    # RosRaspiCheckPower.pyとのトピックのために追加
    #
    rospy.Subscriber("/raspi_power", Float32, powerCallback, queue_size=10)
    shellCallback.pub = rospy.Publisher('/raspi_shell', String, queue_size=2)
    rc.setCallbackShell(shellCallback)
    #
    # メインループはＧＵＩで終了まで実行
    #
    rc.guiApp()
    print("リモコンＧＵＩが終了しました。")


#------------------------------------------------------------------------------
# このファイルから起動時のみ実行する
#------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
