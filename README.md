# 空間IDライブラリ（Python版）

## 概要
- 任意の座標を空間IDに変換するライブラリです。
- 空間ID仕様については以下のリンクを参照して下さい。
<!--[Digital Architecture Design Center 4次元時空間情報基盤 ガイドライン](https://www.ipa.go.jp/digital/architecture/project/autonomousmobilerobot/3dspatial_guideline.html)-->
<p><a href="https://www.ipa.go.jp/digital/architecture/project/autonomousmobilerobot/3dspatial_guideline.html" target="_blank">Digital Architecture Design Center 4次元時空間情報基盤 ガイドライン</a></p>

## 空間IDライブラリ利用方法
1. 下記ディレクトリに移動します。
```
cd src
```
2. pipコマンドでインストールを実行します。
```
pip install .
```
※pipコマンドはpythonの実行環境に合わせます。

3. APIをimportして実行します。
```
import SpatialId.xxx
```
importと呼び出しの例はexamples/examples_api.pyにあります。
APIの詳細についてはdocsフォルダ配下にAPI仕様書があります。

## 注意事項
* ライブラリの入力可能な緯度の最大、最小値は「±85.0511287798」とします。
* 精度レベルの指定範囲は、0から35とします。
* 経度の限界値は±180ですが、180と-180は同じ個所を指すこととZFXY形式のインデックスの考え方により、180はライブラリ内部では-180として扱われます。(180の入力は可能とします。)

## 前提ソフトウェア
- 前提ソフトウェア
    - pybullet  
        - バージョン:3.2.5
        - pythonバージョン:&gt;=3.4
        - 確認日:2022/6/7
        - 用途:円柱と空間ボクセルの衝突確認に使用します
    - pyproj
        - バージョン:3.3.1
        - pythonバージョン:&gt;=3.8
        - 確認日:2022/6/21
        - 座標変換に使用します
    - numpy  
        - バージョン:1.22.4 
        - pythonバージョン:&gt;=3.8
        - 確認日:2022/6/21
        - 行列演算に使用します
    - scikit-spatial  
        - バージョン:6.4.0
        - pythonバージョン:&gt;=3.7
        - 確認日:2022/6/21
        - 線分の演算に使用します
    - pyshp
        - バージョン:2.3.0
        - pythonバージョン:&gt;=3.8
        - 確認日:2022/7/14
        - 用途:Shapefileの読み込みに使用します。

## 問合せ及び要望に関して
- 本リポジトリは現状は主に配布目的の運用となるため、IssueやPull Requestに関しては受け付けておりません。
- 今後対応していく予定ですが、現状Issue等をいただいてもお返事が遅くなる場合がありますのでご了承ください。

## ライセンス
- 本リポジトリはMITライセンスで提供されています。
- ソースコードおよび関連ドキュメントの著作権はデジタル庁に帰属します。

## 免責事項
- 本リポジトリの内容は予告なく変更・削除する可能性があります。
- 本リポジトリの利用により生じた損失及び損害等について、いかなる責任も負わないものとします。
