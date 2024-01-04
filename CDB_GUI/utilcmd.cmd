pyuic5 mainWin.ui -o mainWin_ui.py

pyinstaller -D cdb_main.py -i ./rc/main.ico

pip install pyqt5-tools

pip install pyqt5-stubs

@REM 新建环境

conda remove -n xxxxx(名字) --all

conda update -n base -c defaults conda

conda create -n can_dev

conda activate can_dev

conda install python

pip install pyqt5 numpy matplotlib canlib pyinstaller pyqt5-tools pyqt5-stubs
