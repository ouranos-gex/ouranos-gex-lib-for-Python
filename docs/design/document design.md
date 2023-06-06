# 設計資料構成

各設計資料に記載されている内容は下記の通り。

|ファイル名|概要|対応モジュール|
|-|-|-|
|cylinders.md|指定範囲の空間ID変換（円柱）|SpatialId.shape.cylinders module|
|point.md|地理座標から空間IDへの変換(点群)<br> 空間IDから地理座標または投影座標への変換<br>地理座標から投影座標への変換<br>投影座標から地理座標への変換<br>精度の入力制限の確認|SpatialId.shape.point module|
|polygons.md|指定範囲の空間ID変換（ポリゴン）<br>指定範囲の空間ID変換（線分）|SpatialId.shape.polygons module|
|shapefile.md|Shapefileからの空間ID変換|SpatialId.io.shapefile module|
|shifting_spatial_id.md|指定の数値分、移動した場合の空間IDを取得<br>空間IDの面に直接、接している6個の空間IDの取得<br>空間IDの水平方向の周囲、一周分の8個の空間IDを取得<br>空間IDを囲う26個の空間IDを取得|SpatialId.operated.shifting_spatial_id module|
|document design.md|本書|-|

