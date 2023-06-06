# 設計資料    

本資料では同モジュール内で提供される下記APIについて記載をする。  
- Shapefileからの空間ID変換

<div style="page-break-before:always"></div>

## Shapefileからの空間ID変換

### 更新履歴 
<table border=1>
<header>
<td width=13%>
版数
</td>
<td width=10%>
日付
</td>
<td>
概要
</td>
<td width=18%>
更新者
</td>
</header>
<tr>
<td>0.01</td>
<td>2022/7/29</td>
<td>新規作成</td>
<td>大野</td>
</tr>
<tr>
<td>0.02</td>
<td>2022/8/15</td>
<td>レビューコメントを受け修正
    <ul>
        <li>処理概要を詳細化</li>
        <li>リストスタイルの修正</li>
        <li>誤字修正</li>
    </ul>
</td>
<td>大野</td>
</tr>
</table>

<div style="page-break-before:always"></div>

### 処理概要
Shapefileを読み込み、Shapefileに1レコード毎に「座標情報を空間IDに変換した空間IDリスト」、「フィールド定義情報」、「ペイロード」の組を返却するジェネレータを返却する。

### 処理順序

1. 座標の変換オブジェクトの作成  
ユーザの入力したCRSをWGS84のCRS(EPSG:4326)に変換するオブジェクトを作成する。

1. Shapefile読み込み  
shapefile.Readerオブジェクトを作成する。

1. フィールド定義情報取得  
shapefile.Readerオブジェクトからフィールド定義情報を取得する。
この際、取得するリストの1番目の要素(DeletionFlag)以外をフィールド定義情報として保持する。

1. ループ処理の開始  
shapefile.Readerのイテレータを使用して1レコードずつ読み込む。
    <ol style="list-style-type: upper-roman">
        <li>入力座標をWGS84に変換<br>
        レコードのshape.points及びzから座標を取得し、WGS84の座標に変換し座標リストとする。</li>
        <li>
        shapeTypeによって分岐<br>
        <ul>
            <li>MULTIPATCHの場合<br>
                shape.partsの要素でループ<br>
                <ol style="list-style-type: lower-roman">
                    <li>
                        現在のループ位置で分岐
                        <ul>
                            <li>末尾以外の場合<br>
                            座標取得の開始位置を現在のpartsの要素(現在インデックスをiとした場合parts[i])、終了位置を次のpartsの要素(現在インデックスをiとした場合parts[i+1])とする。</li>
                            <li>末尾の場合<br>
                            座標取得の開始位置を現在のpartsの要素(現在インデックスをiとした場合parts[i])、終了位置をpointsの末尾(リストスライスの終了位置指定なし)とする。</li>
                        </ul>
                    </li>
                    <li>座標リストから開始位置、終了位置のスライスを部品座標リストとして取り出す。</li>
                    <li>
                        現在の要素のpartTypeで分岐
                        <ul>
                            <li>TRIANGLE_FANの場合<br>
                                1から部品座標リストの長さ-2までループして(現在インデックスを仮にiとする)、部品座標リスト[0]、部品座標リスト[i]、部品座標リスト[i+1]の3つの座標の組み合わせのリストを作成する。
                            </li>
                            <li>TRIANGLE_STRIPの場合<br>
                                0から部品座標リストの長さ-3までループして(現在インデックスを仮にiとする)、部品座標リスト[i]、部品座標リスト[i+1]、部品座標リスト[i+2]の3つの座標の組み合わせのリストを作成する。
                            </li>
                            <li>その他の場合<br>
                                例外を送出する。
                            </li>
                        </ul>
                    </li>
                    <li>
                        座標の組み合わせリストをTriangleオブジェクトのリストに変換する。
                    </li>
                    <li>
                        Triangleオブジェクトのリストを入力として、「指定範囲の空間ID変換（ポリゴン）」を実行する。
                    </li>
                    <li>
                        現在のレコードの属性情報(ペイロード)を取得する。
                    </li>
                    <li>
                        「指定範囲の空間ID変換（ポリゴン）」の出力(空間IDリスト)、フィールド定義情報、ペイロードをジェネレータの出力として返却する。
                    </li>
                </ol>
            </li>
            <li>上記以外の場合<br>
                例外を送出する。
            </li>
        </ul>
        </li>
    </ol>

<div style="page-break-before:always"></div>

## Shapefileからの空間ID変換(一括取得)

### 更新履歴 
<table border=1>
<header>
<td width=13%>
版数
</td>
<td width=10%>
日付
</td>
<td>
概要
</td>
<td width=18%>
更新者
</td>
</header>
<tr>
<td>0.01</td>
<td>2022/7/29</td>
<td>新規作成</td>
<td>大野</td>
</tr>
<tr>
<td>0.02</td>
<td>2022/8/15</td>
<td>レビューコメントを受け修正
    <ul>
        <li>処理概要を詳細化</li>
        <li>リストスタイルの修正</li>
        <li>誤字修正</li>
        <li>改ページの追加</li>
    </ul>
</td>
<td>大野</td>
</tr>
</table>

<div style="page-break-before:always"></div>

### 処理概要
Shapefileを読み込み、Shapefileに含まれる全レコードの情報を一括で空間ID変換して返却する。

本処理で返却される情報は以下である。
<ul>
<li>「レコード毎の座標情報を変換した空間IDリスト」のリスト(空間IDリスト)</li>
<li>フィールド定義情報</li>
<li>ペイロードのリスト(ペイロードリスト)</li>
</ul>

### 処理順序

1. 座標の変換オブジェクトの作成  
ユーザの入力したCRSをWGS84のCRS(EPSG:4326)に変換するオブジェクトを作成する。

1. Shapefile読み込み  
shapefile.Readerオブジェクトを作成する。

1. フィールド定義情報取得  
shapefile.Readerオブジェクトからフィールド定義情報を取得する。
この際、取得するリストの1番目の要素(DeletionFlag)以外をフィールド定義情報として保持する。

1. 返却用の空間IDリストを作成する。

1. Shapefileレコード処理ループの開始  
shapefile.Readerのイテレータを使用して1レコードずつ読み込む。
    <ol style="list-style-type: upper-roman">
        <li>入力座標をWGS84に変換<br>
        レコードのshape.points及びzから座標を取得し、WGS84の座標に変換し座標リストとする。</li>
        <li>
        shapeTypeによって分岐<br>
        <ul>
            <li>MULTIPATCHの場合<br>
                shape.partsの要素でループ<br>
                <ol style="list-style-type: lower-roman">
                    <li>
                        現在のループ位置で分岐
                        <ul>
                            <li>末尾以外の場合<br>
                            座標取得の開始位置を現在のpartsの要素(現在インデックスをiとした場合parts[i])、終了位置を次のpartsの要素(現在インデックスをiとした場合parts[i+1])とする。</li>
                            <li>末尾の場合<br>
                            座標取得の開始位置を現在のpartsの要素(現在インデックスをiとした場合parts[i])、終了位置をpointsの末尾(リストスライスの終了位置指定なし)とする。</li>
                        </ul>
                    </li>
                    <li>座標リストから開始位置、終了位置のスライスを部品座標リストとして取り出す。</li>
                    <li>
                        現在の要素のpartTypeで分岐
                        <ul>
                            <li>TRIANGLE_FANの場合<br>
                                1から部品座標リストの長さ-2までループして(現在インデックスを仮にiとする)、部品座標リスト[0]、部品座標リスト[i]、部品座標リスト[i+1]の3つの座標の組み合わせのリストを作成する。
                            </li>
                            <li>TRIANGLE_STRIPの場合<br>
                                0から部品座標リストの長さ-3までループして(現在インデックスを仮にiとする)、部品座標リスト[i]、部品座標リスト[i+1]、部品座標リスト[i+2]の3つの座標の組み合わせのリストを作成する。
                            </li>
                            <li>その他の場合<br>
                                例外を送出する。
                            </li>
                        </ul>
                    </li>
                    <li>
                        座標の組み合わせリストをTriangleオブジェクトのリストに変換する。
                    </li>
                    <li>
                        Triangleオブジェクトのリストを入力として、「指定範囲の空間ID変換（ポリゴン）」を実行する。
                    </li>
                    <li>
                        「指定範囲の空間ID変換（ポリゴン）」の出力(空間IDリスト)を空間IDリストに追加する。
                    </li>
                    <li>
                        現在のレコードの属性情報(ペイロード)を取得し、ペイロードリストに追加する。
                    </li>
                </ol>
            </li>
            <li>上記以外の場合<br>
                例外を送出する。
            </li>
        </ul>
        </li>
    </ol>

1. 空間IDリスト、フィールド定義情報、ペイロードリストを返却する。

<div style="page-break-before:always"></div>

## Shapefileからの空間ID変換(一括取得、ペイロード集約あり)

### 更新履歴 
<table border=1>
<header>
<td width=13%>
版数
</td>
<td width=10%>
日付
</td>
<td>
概要
</td>
<td width=18%>
更新者
</td>
</header>
<tr>
<td>0.01</td>
<td>2022/7/29</td>
<td>新規作成</td>
<td>大野</td>
</tr>
<tr>
<td>0.02</td>
<td>2022/8/15</td>
<td>レビューコメントを受け修正
    <ul>
        <li>処理概要を詳細化</li>
        <li>リストスタイルの修正</li>
        <li>誤字修正</li>
        <li>改ページの追加</li>
    </ul>
</td>
<td>大野</td>
</tr>
</table>

<div style="page-break-before:always"></div>

### 処理概要
Shapefileを読み込み、Shapefileに含まれる全レコードの情報を一括で空間ID変換して返却する。返却する際にペイロードを空間ID毎に集約したペイロード辞書形式で返却する。

本処理で返却される情報は以下である。
<ul>
<li>「レコード毎の座標情報を変換した空間IDのリスト」のリスト(空間IDリスト)</li>
<li>フィールド定義情報</li>
<li>ペイロード辞書</li>
</ul>

ペイロード辞書はキーを空間ID、値を空間IDに対応するペイロードのリストとしたデータ構造である。
処理の実行時に集約関数が指定されている場合はペイロード辞書の値に対して集約関数を適用し、その結果をペイロード辞書の値として置き換える。

### 処理順序

1. 座標の変換オブジェクトの作成  
ユーザの入力したCRSをWGS84のCRS(EPSG:4326)に変換するオブジェクトを作成する。

1. Shapefile読み込み  
shapefile.Readerオブジェクトを作成する。

1. フィールド定義情報取得  
shapefile.Readerオブジェクトからフィールド定義情報を取得する。
この際、取得するリストの1番目の要素(DeletionFlag)以外をフィールド定義情報として保持する。

1. 返却用の空間IDリストを作成する。

1. Shapefileレコード処理ループの開始  
shapefile.Readerのイテレータを使用して1レコードずつ読み込む。
    <ol style="list-style-type: upper-roman">
        <li>入力座標をWGS84に変換<br>
        レコードのshape.points及びzから座標を取得し、WGS84の座標に変換し座標リストとする。</li>
        <li>
        shapeTypeによって分岐<br>
        <ul>
            <li>MULTIPATCHの場合<br>
                shape.partsの要素でループ<br>
                <ol style="list-style-type: lower-roman">
                    <li>
                        現在のループ位置で分岐
                        <ul>
                            <li>末尾以外の場合<br>
                            座標取得の開始位置を現在のpartsの要素(現在インデックスをiとした場合parts[i])、終了位置を次のpartsの要素(現在インデックスをiとした場合parts[i+1])とする。</li>
                            <li>末尾の場合<br>
                            座標取得の開始位置を現在のpartsの要素(現在インデックスをiとした場合parts[i])、終了位置をpointsの末尾(リストスライスの終了位置指定なし)とする。</li>
                        </ul>
                    </li>
                    <li>座標リストから開始位置、終了位置のスライスを部品座標リストとして取り出す。</li>
                    <li>
                        現在の要素のpartTypeで分岐
                        <ul>
                            <li>TRIANGLE_FANの場合<br>
                                1から部品座標リストの長さ-2までループして(現在インデックスを仮にiとする)、部品座標リスト[0]、部品座標リスト[i]、部品座標リスト[i+1]の3つの座標の組み合わせのリストを作成する。
                            </li>
                            <li>TRIANGLE_STRIPの場合<br>
                                0から部品座標リストの長さ-3までループして(現在インデックスを仮にiとする)、部品座標リスト[i]、部品座標リスト[i+1]、部品座標リスト[i+2]の3つの座標の組み合わせのリストを作成する。
                            </li>
                            <li>その他の場合<br>
                                例外を送出する。
                            </li>
                        </ul>
                    </li>
                    <li>
                        座標の組み合わせリストをTriangleオブジェクトのリストに変換する。
                    </li>
                    <li>
                        Triangleオブジェクトのリストを入力として、「指定範囲の空間ID変換（ポリゴン）」を実行する。
                    </li>
                    <li>
                        「指定範囲の空間ID変換（ポリゴン）」の出力を空間IDリストに追加する。
                    </li>
                    <li>
                        現在のレコードの属性情報(ペイロード)を取得し、ペイロードリストに追加する。
                    </li>
                </ol>
            </li>
            <li>上記以外の場合<br>
                例外を送出する。
            </li>
        </ul>
        </li>
    </ol>

1. 空間IDリスト、ペイロードリストからキーを空間ID、値を空間IDに対応するペイロードのリストとした辞書を作成する。<br>
    <ol style="list-style-type: upper-roman">
        <li>空のペイロード辞書を作成する</li>
        <li>ループで先頭から順に末尾の要素まで空間IDリスト、ペイロードリストからそれぞれ要素を取得する。<br>
            ループで空間IDリストから取得したリストの要素(空間IDxとする)を処理する。<br>
            <ol style="list-style-type: lower-roman">
                <li>
                    空間IDxがペイロード辞書のキーとして存在しない場合、ペイロード辞書にキーを空間IDx、値を空のリストとする。</li>
                <li>ペイロード辞書の空間IDxの値のリストにペイロードを追加する。</li>
            </ol>
        </li>
    </ol>

1. 集約関数が指定されている場合、ペイロード辞書のキー、値でループする。<br>
    値を集約関数に入力し、出力をペイロード辞書のキーに対応する値とする。

1. 空間IDリスト、フィールド定義情報、ペイロード辞書を返却する。

<div style="page-break-before:always"></div>

### 制約事項


## 使用ライブラリ

### 更新履歴 
<table border=1>
<header>
<td width=13%>
版数
</td>
<td width=10%>
日付
</td>
<td>
概要
</td>
<td width=18%>
更新者
</td>
</header>
<tr>
<td>0.01</td>
<td>2022/7/29</td>
<td>新規作成</td>
<td>大野</td>
</tr>
</table>

- pyproj  
    - バージョン:3.3.1
    - pythonバージョン:&gt;=3.8
    - 確認日:2022/5/30
    - 用途:空間座標系の変換に使用する。
- pyshp
    - バージョン:2.3.0
    - pythonバージョン:&gt;=3.8
    - 確認日:2022/7/14
    - 用途:Shapefileの読み込みに使用する。

