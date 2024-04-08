# 環境構築

## Install Python

https://www.python.org/downloads/windows/

<img src="./img/python_install.png" width="60%">

Download Windows installer (64-bit)

<img src="./img/python_install2.png" width="60%">

Add python.exe to PATH にチェック

インストール後再起動

## 作業ディレクトリの作成
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

## Install Mujoco

~~~
pip install mujoco
~~~