# 環境構築

## Install Python

https://www.python.org/downloads/windows/

<img src="./img/python_install.png" width="60%">

Download Windows installer (64-bit)

<img src="./img/python_install2.png" width="60%">

Add python.exe to PATH にチェック

インストール後再起動

## 作業ディレクトリの作成

ターミナル（コマンドプロンプト）を開く
~~~
mkdir workspace
~~~

## 仮想環境の作成
~~~
cd workspace
python -m venv manipulator
~~~

## Activate
~~~
env\Scripts\activate
~~~

## 必要なライブラリのインストール

~~~
pip install mujoco
pip install matplotlib
~~~

##  動作確認 

Quiz ディレクトリをそのまま自分の環境にコピー

Control.py を実行
~~~
cd ~/Quiz
python Control.py
~~~

↓の画面が表示されたら完了

<img src="./img/test.png" width="60%">
