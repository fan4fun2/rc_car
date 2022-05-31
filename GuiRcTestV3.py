#!/usr/bin/env python
# -*- coding: utf-8 -*-

#==============================================================================
# ラジコン・カーをＧＵＩでテストするためのリモコンプログラム
#                                                       RosRemoCon.py Ver.3.1
#                                                       fan4fun2rc 22 Mar. 2022
#------------------------------------------------------------------------------
#　V1:ラジコン・カーの速度情報とパラメータをTkinterのＧＵＩを使用して視覚的にテスト
#　   できるようにしたリモコンプログラムで、ＲＯＳとのインターフェースはRosBrdgGui.py
#　   が橋渡しする。
#　V2:目標速度制御を行うときのＰＷＭにオフセットを追加したため、ＧＵＩも変更となり
#　   プロトコルにも互換がないためメジャー・バージョンの変更とした。
#　V3:ラズパイ本体で電圧監視するプログラムRosRaspiCheckPower.pyとＧＵＩ連携する
#　   ために、現在のラズパイ電圧を表示するテキストボックスと電圧低下時にラズパイを
#　   シャットダウンするためのボタンを追加する。
#　   併せて、緊急停止用の目標速度を０[mm/s]とＰＷＭを０[%]にするボタンを追加する。
#
#【仕様】
#　・現在の速度情報はラズパイからＲＯＳブリッジ経由で関数がコールされる
#　・駆動指示は通常の目標速度とモーターへのＰＷＭ比率をラジオボタンで切替え可能とする
#　・テキストボックスに設定された駆動情報は左右別々または左右同時にボタンを設ける
#　・駆動指示のボタンが押されるとＲＯＳブリッジに通知する関数を登録できるようにする
#　・ラズパイが実行するＰＩＤ速度制御のパラメータを左右独立に設定できるようにする
#　・テキストボックスに設定された制御係数は左右別々または左右同時にボタンを設ける
#　・制御係数のボタンが押されるとＲＯＳブリッジに通知する関数を登録できるようにする
#　・ラズパイのバッテリ電圧を表示するテキストボックスと受信処理する関数を追加する
#　・シャットダウンボタンが押されるとＲＯＳブリッジに通知する関数を登録できるようにする
#　・緊急停止用のボタンを２つ追加して目標速度制御またはＰＷＭ直接駆動で停止させる
#
#【関数概要】
#　１．左右にGUIのパーツを配置するフレームを作成する関数：labelFrame(f, tx, col)
#　２．中央にGUIのパーツを配置するフレームを作成する関数：centerFrame(f)
#　３．表示や設定するテキストボックスとボタンを作成する関数：editButton(f, r, c, t)
#　４．目標速度とＰＷＭを選択するラジオボタンを作成する関数：radioButton(f, r, c)
#　５．緊急停止用のボタンを作成する関数：stopButton(f, r, c)
#　６．電圧表示とシャットダウン・ボタンを作成する関数：shutButton(f, r, c)
#　７．駆動情報送信ボタンが押されたときにメイン処理に通知する関数：sndDrive(lr)
#　８．制御係数送信ボタンが押されたときにメイン処理に通知する関数：sndParam(lr)
#　９．GUIを構成する実質のメイン処理関数：guiApp()
#　10．現在速度のテキストボックスに表示するセッター関数：setSpeed(sp)
#　11．ラズパイの電圧をテキストボックスに表示するセッター関数：setPower(f)
#　12．駆動情報が変更されたときに通知する関数を設定する関数：setCallbackDrive(fn)
#　13．制御係数が変更されたときに通知する関数を設定する関数：setCallbackParam(fn)
#　14．シャットダウンされたときに通知する関数を設定する関数：setCallbackShell(fn)
#　15．駆動情報がコールバック設定されて無い時にprintで代用する関数：printd(lr, val)
#　16．制御係数がコールバック設定されて無い時にprintで代用する関数：printp(s,l,p,i,d)
#　17．シェルがコールバック設定されて無い時にprintで代用する関数：prints(s)
#　18．メイン処理関数：main()
#
#【リビジョン・ヒストリ】
#　・V1.0-2022/03/06:コーディング開始
#　・V2.0-2022/03/14:目標速度制御を行うときのＰＷＭにオフセットを追加
#　・V3.0-2022/03/16:ラズパイの電圧表示とシャットダウンボタン、緊急停止ボタンを追加
#　・V3.0-2022/03/22:ラズパイから５秒間以上電圧の更新がなければ、0.00[V]を表示
#==============================================================================

#------------------------------------------------------------------------------
#　インポートするライブラリの宣言
#------------------------------------------------------------------------------
import Tkinter as tk    # python3系では、<import tkinter as tk>
import ttk              # python3系では、<from tkinter import ttk>
import tkMessageBox     # python3系では、<from tkinter import messagebox>
import threading
import time

#==============================================================================
# １．左右にGUIのパーツを配置するフレームを作成する関数：labelFrame(f, tx, col)
#   ＜引数  ｆ：フレームを配置するフレームのID＞
#   ＜引数  tx：フレームに表示するテキスト＞
#   ＜引数 col：フレームを配置するグリッドの列番号＞
#   ＜返値  lf：作成したフレームのID＞
#------------------------------------------------------------------------------
#　・左右にGUIのパーツを配置するフレームを作成
#　・左右を区別するために表示するテキストと列情報を引数とし
#　　作成したフレームのIDを返り値としてリターンする
#==============================================================================
def labelFrame(f, tx, col):
    lf = tk.LabelFrame(f, width=250, height=0, text=tx, bg="white", bd=5,
                            relief="groove", labelanchor="s", padx=5, pady=5)
    lf.grid(row=0, column=col, sticky=tk.NSEW)
    #
    # ウィンドを拡大縮小時に一緒に伸縮する
    #  行と列を指定する
    #
    lf.columnconfigure(0, weight=1)
    lf.columnconfigure(1, weight=1)
    lf.columnconfigure(2, weight=1)
    lf.columnconfigure(3, weight=1)
    lf.columnconfigure(4, weight=1)
    lf.rowconfigure(0, weight=1)
    lf.rowconfigure(1, weight=1)
    lf.rowconfigure(2, weight=1)
    lf.rowconfigure(3, weight=1)
    return lf


#==============================================================================
# ２．中央にGUIのパーツを配置するフレームを作成する関数：centerFrame(f)
#   ＜引数  ｆ：フレームを配置するフレームのID＞
#   ＜返値  cf：作成したフレームのID＞
#------------------------------------------------------------------------------
#　・中央にGUIのパーツを配置するフレームを作成
#==============================================================================
def centerFrame(f):
    cf = tk.LabelFrame(f, width=150, height=0, text="設定ボタン", bg="white",
                        bd=5, relief="groove", labelanchor="s", padx=5, pady=5)
    cf.grid(row=0, column=1, sticky=tk.NSEW)
    cf.columnconfigure(0, weight=1)     # ウィンドを拡大縮小時に一緒に伸縮する
    cf.columnconfigure(1, weight=1)     # 　行と列を指定する
    cf.rowconfigure(0, weight=1)
    cf.rowconfigure(1, weight=1)
    cf.rowconfigure(2, weight=1)
    cf.rowconfigure(3, weight=1)
    return cf


#==============================================================================
# ３．表示や設定するテキストボックスとボタンを作成する関数：editButton(f, r, c, t)
#   ＜引数  f：部品を配置するフレームのID＞
#   ＜引数  r：テキストボックスを配置する行を指定する＞
#   ＜引数  c：テキストボックスを配置する列を指定する＞
#   ＜引数  t：作成したラベルに表示する文字列を指定する＞
#   ＜返値 ev：作成したテキストボックスの文字列変数へのオブジェクト＞
#------------------------------------------------------------------------------
#　・エディットボックスの作成
#　・関数内関数でボタンがクリックされたときにエディットボックスを０クリアする
#　・車エディットボックスのラベルを表示するボタンの作成
#==============================================================================
def editButton(f, r, c, t):
    #
    # エディットボックスの作成
    #
    ev = tk.StringVar(value="0")
    tk.Entry(f, textvariable=ev, justify=tk.CENTER, width=6, font=("", 12)
                ).grid(row=r, column=c, sticky=tk.NSEW)
    #
    # 関数内関数でボタンがクリックされたときにエディットボックスを０クリアする
    #
    def fn():
        ev.set(str(0))
        bt.focus_set()
    #
    # エディットボックスのラベルを表示するボタンの作成
    #
    bt = tk.Button(f, text=t, justify=tk.CENTER, width=6, height=2, command=fn)
    bt.grid(row=r+1, column=c, sticky=tk.NSEW)
    return ev


#==============================================================================
# ４．目標速度とＰＷＭを選択するラジオボタンを作成する関数：radioButton(f, r, c)
#   ＜引数  f：部品を配置するフレームのID＞
#   ＜引数  r：テキストボックスを配置する行を指定する＞
#   ＜引数  c：テキストボックスを配置する列を指定する＞
#   ＜返値 rv：作成したラジオボタンのウィジット変数へのオブジェクト＞
#------------------------------------------------------------------------------
#　・ラジオボタンのフレームを作成
#　・ラジオボタンの作成
#==============================================================================
def radioButton(f, r, c):
    #
    # ラジオボタンのフレームを作成
    #
    rf = tk.LabelFrame(f, width=0, height=0, text="駆動選択", font=("", 8),
                relief="groove", labelanchor="n")
    rf.grid(row=r, rowspan=2, column=c, sticky=tk.NSEW)
    rf.columnconfigure(0, weight=1)     # ウィンドを拡大縮小時に一緒に伸縮する
    rf.rowconfigure(0, weight=1)        # 　行と列を指定する
    rf.rowconfigure(1, weight=1)
    #
    # ラジオボタンの作成
    #
    rv = tk.StringVar(value="T")
    tk.Radiobutton(rf, text='目標速度', value="T", variable=rv, bg="white",
                font=("", 8)).grid(row=0, column=0, sticky=tk.NSEW)
    tk.Radiobutton(rf, text=' Ｐ  Ｗ  Ｍ ', value="W", variable=rv, bg="white",
                font=("", 8)).grid(row=1, column=0, sticky=tk.NSEW)
    return rv


#==============================================================================
# ５．緊急停止用のボタンを作成する関数：stopButton(f, r, c)
#   ＜引数  f：部品を配置するフレームのID＞
#   ＜引数  r：フレームを配置する行を指定する＞
#   ＜引数  c：フレームを配置する列を指定する＞
#------------------------------------------------------------------------------
#　・緊急停止ボタンを２つ用のフレームを作成
#　・関数内関数でボタンがクリックされたときに、両車輪の目標速度に0mm/sを送信する
#　・両車輪の目標速度を０とする緊急停止ボタンの作成
#　・関数内関数でボタンがクリックされたときに、両車輪のＰＷＭに0%を送信する
#　・両車輪のＰＷＭを０とする緊急停止ボタンの作成
#==============================================================================
def stopButton(f, r, c):
    #
    # 緊急停止ボタンを２つ用のフレームを作成
    #
    bf = tk.LabelFrame(f, width=0, height=0, text="緊急停止", font=("", 8),
                bg="red", relief="groove", labelanchor="n")
    bf.grid(row=r, rowspan=2, column=c, sticky=tk.NSEW)
    bf.columnconfigure(0, weight=1)     # ウィンドを拡大縮小時に一緒に伸縮する
    bf.rowconfigure(0, weight=1)        # 　行と列を指定する
    bf.rowconfigure(1, weight=1)
    #
    # 関数内関数でボタンがクリックされたときに、両車輪の目標速度に0mm/sを送信する
    #
    def ft():
        sndDrive.callbackDrive('LT', 0)
        guiApp.ltv.set(0)
        guiApp.lrv.set('T')
        sndDrive.callbackDrive('RT', 0)
        guiApp.rtv.set(0)
        guiApp.rrv.set('T')
        bf.focus_set()
    #
    # 両車輪の目標速度を０とする緊急停止ボタンの作成
    #
    tk.Button(bf, text='目標速度', justify=tk.CENTER, width=6, height=1,
        bg="yellow", command=ft, font=("", 8)).grid(row=0, column=0, sticky=tk.NSEW)
    #
    # 関数内関数でボタンがクリックされたときに、両車輪のＰＷＭに0%を送信する
    #
    def fw():
        sndDrive.callbackDrive('LW', 0)
        guiApp.lwv.set(0)
        guiApp.lrv.set('W')
        sndDrive.callbackDrive('RW', 0)
        guiApp.rwv.set(0)
        guiApp.rrv.set('W')
        bf.focus_set()
    #
    # 両車輪のＰＷＭを０とする緊急停止ボタンの作成
    #
    tk.Button(bf, text='ＰＷＭ', justify=tk.CENTER, width=6, height=1,
        bg="yellow", command=fw, font=("", 8)).grid(row=1, column=0, sticky=tk.NSEW)


#==============================================================================
# ６．電圧表示とシャットダウン・ボタンを作成する関数：shutButton(f, r, c)
#   ＜引数  f：部品を配置するフレームのID＞
#   ＜引数  r：フレームを配置する行を指定する＞
#   ＜引数  c：フレームを配置する列を指定する＞
#   ＜返値 pv：作成したテキストボックスのウィジット変数へのオブジェクト＞
#   ＜返値 pf：作成したテキストボックスへのオブジェクト＞
#------------------------------------------------------------------------------
#　・ラズパイ用の電圧表示とシャットダウン・ボタンを配置するフレームを作成
#　・ラズパイの電圧表示用のテキストボックスを作成
#　・関数内関数でシャットダウン確認ダイアログの表示
#　・関数内関数でボタンがクリックされたときに、シャットダウン・コマンドを送信する
#　・シャットダウン・ボタンの作成
#==============================================================================
def shutButton(f, r, c):
    #
    # ラズパイ用の電圧表示とシャットダウン・ボタンを配置するフレームを作成
    #
    rf = tk.LabelFrame(f, width=0, height=0, text="ラズパイ", font=("", 8),
                bg="red", relief="groove", labelanchor="n")
    rf.grid(row=r, rowspan=2, column=c, sticky=tk.NSEW)
    rf.columnconfigure(0, weight=1)     # ウィンドを拡大縮小時に一緒に伸縮する
    rf.rowconfigure(0, weight=1)        # 　行と列を指定する
    rf.rowconfigure(1, weight=1)
    #
    # ラズパイの電圧表示用のテキストボックスを作成
    #
    pv = tk.StringVar(value="0.00[V]")
    pe = tk.Entry(rf, textvariable=pv, justify=tk.CENTER, width=6,
                font=("", 12), bg="silver")
    pe.grid(row=0, column=0, sticky=tk.NSEW)
    #
    # 関数内関数でボタンがクリックされたときに、シャットダウン・コマンドを送信する
    #
    def fs():
        rf.focus_set()
        ret = tkMessageBox.askokcancel("確認", "ラズパイをシャットダウンしますか？")
        if ret == True:
            shutButton.callbackShell('sudo shutdown -h now')
        else:
            print('ラズパイのシャットダウンをキャンセルしました。')
        
    #
    # シャットダウン・ボタンの作成
    #
    tk.Button(rf, text='シャット\nダウン', justify=tk.CENTER, width=6, height=1,
                bg="yellow", command=fs, font=("", 8)
                ).grid(row=1, column=0, sticky=tk.NSEW)
    return pv, pe


#==============================================================================
# ７．駆動情報送信ボタンが押されたときにメイン処理に通知する関数：sndDrive(lr)
#   ＜引数 lr：押されたボタンを文字列[L/R/B]で表す＞
#------------------------------------------------------------------------------
#　・左右の情報とラジオボタンの情報を文字列でミックスし、テキストボックスの値と
#　　共に指定のコールバック関数を通じてメイン処理に通知する
#　・フォーカスをテキストボックスから押されてボタンに移動
#==============================================================================
def sndDrive(lr):
    #
    # 左右の情報とラジオボタンの情報を文字列でミックスし、テキストボックスの値と
    #　共に指定のコールバック関数を通じてメイン処理に通知する
    #
    if lr == 'L' or lr == 'B':
        if guiApp.lrv.get() == 'T':
            sndDrive.callbackDrive('LT', int(guiApp.ltv.get()))
        else:
            sndDrive.callbackDrive('LW', int(guiApp.lwv.get()))
    if lr == 'R' or lr == 'B':
        if guiApp.rrv.get() == 'T':
            sndDrive.callbackDrive('RT', int(guiApp.rtv.get()))
        else:
            sndDrive.callbackDrive('RW', int(guiApp.rwv.get()))
    #
    # フォーカスをテキストボックスから押されたボタンに移動
    #
    if lr == 'L':
        guiApp.ul.focus_set()
    elif lr == 'R':
        guiApp.ur.focus_set()
    else:
        guiApp.uc.focus_set()


#==============================================================================
# ８．制御係数送信ボタンが押されたときにメイン処理に通知する関数：sndParam(lr)
#   ＜引数 lr：押されたボタンを文字列[L/R/B]で表す＞
#------------------------------------------------------------------------------
#　・左右の情報とテキストボックスの目標係数、比例係数、積分係数、微分係数を
#　　指定のコールバック関数を通じてメイン処理に通知する
#　・フォーカスをテキストボックスから押されてボタンに移動
#==============================================================================
def sndParam(lr):
    #
    # 左右の情報とテキストボックスの目標係数、比例係数、積分係数、微分係数を
    #　指定のコールバック関数を通じてメイン処理に通知する
    #
    if lr == 'L' or lr == 'B':
        sndParam.callbackParam('L', int(guiApp.llv.get()), int(guiApp.lpv.get()),
            int(guiApp.liv.get()), int(guiApp.ldv.get()), int(guiApp.lov.get()))
    if lr == 'R' or lr == 'B':
        sndParam.callbackParam('R', int(guiApp.rlv.get()), int(guiApp.rpv.get()),
            int(guiApp.riv.get()), int(guiApp.rdv.get()), int(guiApp.rov.get()))
    #
    # フォーカスをテキストボックスから押されたボタンに移動
    #
    if lr == 'L':
        guiApp.ll.focus_set()
    elif lr == 'R':
        guiApp.lr.focus_set()
    else:
        guiApp.lc.focus_set()


#==============================================================================
# ９．GUIを構成する実質のメイン処理関数：guiApp()
#------------------------------------------------------------------------------
#　・メインウィンドを作成
#　・左右と中央にGUIのパーツを配置するフレームを作成
#　・左側の車輪の制御情報を表示するエディットボックスとボタンを配置する
#　・右側の車輪の制御情報を表示するエディットボックスとボタンを配置する
#　・中央のフレームにテキストボックスの設定値を送信するボタンの作成
#　・実行は、ここで停止しGUIのイベント処理を行う
#==============================================================================
def guiApp():
    #
    # メインウィンドを作成
    #
    root = tk.Tk()
    root.title('ラジコン・テスト用リモコン　Ver.3.1')     # 画面のタイトルを設定
    root.geometry('+456+0')                             # 画面を右上に固定
    root.columnconfigure(0, weight=1)
    root.columnconfigure(1, weight=1)
    root.columnconfigure(2, weight=1)
    root.rowconfigure(0, weight=1)
    #
    # 左右と中央にGUIのパーツを配置するフレームを作成
    #
    lf = labelFrame(root, "左車輪の制御状況", 0)
    cf = centerFrame(root)
    rf = labelFrame(root, "右車輪の制御状況", 2)
    #
    # 左側の車輪の制御情報を表示するエディットボックスとボタンを配置する
    #
    guiApp.ltv = editButton(lf, 0, 0, '目標速度\n[mm/s]')
    guiApp.lcv = editButton(lf, 0, 1, '現在速度\n[mm/s]')
    guiApp.lwv = editButton(lf, 0, 2, 'ＰＷＭ\n[%]')
    guiApp.lrv = radioButton(lf, 0, 3)
    stopButton(lf, 0, 4)

    guiApp.llv = editButton(lf, 2, 0, '目標係数\nL[%]')
    guiApp.llv.set(15)
    guiApp.lpv = editButton(lf, 2, 1, '比例係数\nP[%]')
    guiApp.lpv.set(10)
    guiApp.liv = editButton(lf, 2, 2, '積分係数\nI[%]')
    guiApp.liv.set(10)
    guiApp.ldv = editButton(lf, 2, 3, '微分係数\nD[%]')
    guiApp.ldv.set(0)
    guiApp.lov = editButton(lf, 2, 4, 'ＰＷＭ\nオフセット[%]')
    guiApp.lov.set(10)
    #
    # 右側の車輪の制御情報を表示するエディットボックスとボタンを配置する
    #
    guiApp.rtv = editButton(rf, 0, 2, '目標速度\n[mm/s]')
    guiApp.rcv = editButton(rf, 0, 3, '現在速度\n[mm/s]')
    guiApp.rwv = editButton(rf, 0, 4, 'ＰＷＭ\n[%]')
    guiApp.rrv = radioButton(rf, 0, 1)
    guiApp.pwt, guiApp.pwe = shutButton(rf, 0, 0)
    #guiApp.pwt.set('0.00[V]')
    #setPower(0.0)
    #setPower.t = threading.Thread(target=setPower,args=(0.0,))
    setPower.t = threading.Timer(1, setPower,args=(0.0, ))
    setPower.t.start()

    guiApp.rlv = editButton(rf, 2, 0, '目標係数\nL[%]')
    guiApp.rlv.set(15)
    guiApp.rpv = editButton(rf, 2, 1, '比例係数\nP[%]')
    guiApp.rpv.set(10)
    guiApp.riv = editButton(rf, 2, 2, '積分係数\nI[%]')
    guiApp.riv.set(10)
    guiApp.rdv = editButton(rf, 2, 3, '微分係数\nD[%]')
    guiApp.rdv.set(0)
    guiApp.rov = editButton(rf, 2, 4, 'ＰＷＭ\nオフセット[%]')
    guiApp.rov.set(10)
    #
    # 中央のフレームにテキストボックスの設定値を送信するボタンの作成
    #
    guiApp.ul = tk.Button(cf, text='左', justify=tk.CENTER, width=2, height=1,
                            command=lambda:sndDrive('L'))
    guiApp.ul.grid(row=0, column=0, sticky=tk.NSEW)
    guiApp.ur = tk.Button(cf, text='右', justify=tk.CENTER, width=2, height=1,
                            command=lambda:sndDrive('R'))
    guiApp.ur.grid(row=0, column=1, sticky=tk.NSEW)
    guiApp.uc = tk.Button(cf, text='両方', justify=tk.CENTER, width=4, height=1,
                            command=lambda:sndDrive('B'))
    guiApp.uc.grid(row=1, column=0, columnspan=2, sticky=tk.NSEW)

    guiApp.ll = tk.Button(cf, text='左', justify=tk.CENTER, width=2, height=1,
                            command=lambda:sndParam('L'))
    guiApp.ll.grid(row=2, column=0, sticky=tk.NSEW)
    guiApp.lr = tk.Button(cf, text='右', justify=tk.CENTER, width=2, height=1,
                            command=lambda:sndParam('R'))
    guiApp.lr.grid(row=2, column=1, sticky=tk.NSEW)
    guiApp.lc = tk.Button(cf, text='両方', justify=tk.CENTER, width=4, height=1,
                            command=lambda:sndParam('B'))
    guiApp.lc.grid(row=3, column=0, columnspan=2, sticky=tk.NSEW)
    #
    # 実行は、ここで停止しGUIのイベント処理を行う
    #
    #root.update_idletasks()
    #print root.geometry(), root.winfo_width(), root.winfo_height(), root.winfo_x(), root.winfo_y()
    root.mainloop()
    setPower.t.cancel()


#==============================================================================
# 10．現在速度のテキストボックスに表示するセッター関数：setSpeed(sp)
#   ＜引数 sp：更新する速度情報のリスト[L,R]mm/sec＞
#------------------------------------------------------------------------------
#　・他のメイン処理からライブラリとしてインポートされたときに
#　　左右のテキストボックスに速度情報を表示するサポート関数
#==============================================================================
def setSpeed(sp):
    guiApp.lcv.set(str(sp[0]))      # 左速度情報
    guiApp.rcv.set(str(sp[1]))      # 右速度情報


#==============================================================================
# 11．ラズパイの電圧をテキストボックスに表示するセッター関数：setPower(f)
#   ＜引数 f：更新する電圧の浮動小数点数＞
#------------------------------------------------------------------------------
#　・他のメイン処理からライブラリとしてインポートされたときに
#　　ラズパイの電圧表示用テキストボックスに表示するサポート関数
#　・電圧の値によってバックカラーを変更し注意を促す
#==============================================================================
def setPower(f):
    if f == 0.0:
        guiApp.pwe.configure(bg="silver")
    elif f > 4.7:
        guiApp.pwe.configure(bg="aqua")
    elif f > 4.0:
        guiApp.pwe.configure(bg="yellow")
    elif f > 3.6:
        guiApp.pwe.configure(bg="pink")
    else:
        guiApp.pwe.configure(bg="red")
    guiApp.pwt.set("%4.2f[V]" % f)
    #
    # ５秒間関数がコールされなかったら0.00[V]表示に戻るタイマーを設定
    #
    setPower.t.cancel()         # リトリガブルのため一旦キャンセル
    setPower.t = threading.Timer(5, setPower,args=(0.0, ))
    setPower.t.start()
        


#==============================================================================
# 12．駆動情報が変更されたときに通知する関数を設定する関数：setCallbackDrive(fn)
#   ＜引数 fn：通知するコールバック関数＞
#------------------------------------------------------------------------------
#　・他のメイン処理からライブラリとしてインポートされたときに
#　　左右の駆動情報設定値がGUIで変更されたときに通知するコールバック関数を設定する
#==============================================================================
def setCallbackDrive(fn):
    sndDrive.callbackDrive = fn


#==============================================================================
# 13．制御係数が変更されたときに通知する関数を設定する関数：setCallbackParam(fn)
#   ＜引数 fn：通知するコールバック関数＞
#------------------------------------------------------------------------------
#　・他のメイン処理からライブラリとしてインポートされたときに
#　　左右の制御係数設定値がGUIで変更されたときに通知するコールバック関数を設定する
#==============================================================================
def setCallbackParam(fn):
    sndParam.callbackParam = fn


#==============================================================================
# 14．シャットダウンされたときに通知する関数を設定する関数：setCallbackShell(fn)
#   ＜引数 fn：通知するコールバック関数＞
#------------------------------------------------------------------------------
#　・他のメイン処理からライブラリとしてインポートされたときに
#　　シャットダウンが指示されたときに通知するコールバック関数を設定する
#==============================================================================
def setCallbackShell(fn):
    shutButton.callbackShell = fn


#==============================================================================
# 15．駆動情報がコールバック設定されて無い時にprintで代用する関数：printd(lr, val)
#   ＜引数  lr：押されたボタンとラジオボタンをミックスした文字列[LT/LW/RT/RW]＞
#   ＜引数 val：押されたボタンとラジオボタンに対応したテキストボックスの値＞
#------------------------------------------------------------------------------
#　・python2ではprint文をコールバックできないので、ワンクッション関数
#==============================================================================
def printd(lr, val):
    print lr, val


#==============================================================================
# 16．制御係数がコールバック設定されて無い時にprintで代用する関数：printp(s,l,p,i,d,o)
#   ＜引数 s：押されたボタンとラジオボタンをミックスした文字列[L/R]＞
#   ＜引数 l,p,i,d,o：各係数のテキストボックスの値＞
#------------------------------------------------------------------------------
#　・python2ではprint文をコールバックできないので、ワンクッション関数
#==============================================================================
def printp(s,l,p,i,d,o):
    print s, l, p, i, d, o


#==============================================================================
# 17．シェルがコールバック設定されて無い時にprintで代用する関数：prints(s)
#   ＜引数 s：シェルに渡すコマンドの文字列＞
#------------------------------------------------------------------------------
#　・python2ではprint文をコールバックできないので、ワンクッション関数
#==============================================================================
def prints(s):
    print s


#==============================================================================
# 18．メイン処理関数：main()
#------------------------------------------------------------------------------
#　・他のメイン処理からライブラリとしてインポートされたときに
#　　guiApp()のみを呼び出せばよいように、main()からワンクッション置いた
#==============================================================================
def main():
    #
    # コールバック関数が設定されていないときはprint文で代用
    #
    sndDrive.callbackDrive = printd # python2ではprint文をコールバックできないので
    sndParam.callbackParam = printp
    shutButton.callbackShell = prints
    guiApp()

#------------------------------------------------------------------------------
# このファイルから起動時のみ実行する
#------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
