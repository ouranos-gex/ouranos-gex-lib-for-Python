import SpatialId.common.object.enum

import SpatialId.common.object.point
import SpatialId.io.shapefile

import SpatialId.operated.shifting_spatial_id
import SpatialId.shape.cylinders
import SpatialId.shape.point

import SpatialId.shape.polygons

# get_spatial_ids_on_pointsの呼び出し
## get_spatial_ids_on_points(単一の座標,CRSの指定あり)
SpatialId.shape.point.get_spatial_ids_on_points([SpatialId.common.object.point.Point(139.0, 35.0, 256)], 15, 10, 4326)
## get_spatial_ids_on_points(単一の座標,CRSの指定なし)
SpatialId.shape.point.get_spatial_ids_on_points([SpatialId.common.object.point.Point(139.0, 35.0, 256)], 15, 10)
## get_spatial_ids_on_points(複数の座標)
SpatialId.shape.point.get_spatial_ids_on_points(
        [SpatialId.common.object.point.Point(139.0, 35.0, 256), SpatialId.common.object.point.Point(138.5, 35.2, 35)], 15, 10)

# get_point_on_spatial_idの呼び出し
## get_point_on_spatial_id(空間IDの頂点の座標を緯度経度で取得する)
SpatialId.shape.point.get_point_on_spatial_id('20/13/5/23/4', SpatialId.common.object.enum.Point_Option.VERTEX, 4326)
## get_point_on_spatial_id(空間IDの中央の座標をWebメルカトル(xy)で取得する)
SpatialId.shape.point.get_point_on_spatial_id('20/13/5/23/4', SpatialId.common.object.enum.Point_Option.CENTER, 3857)

# convert_point_list_to_projected_point_listの呼び出し
## convert_point_list_to_projected_point_list(Webメルカトルの座標をWGS84の座標に変換する)
SpatialId.shape.point.convert_point_list_to_projected_point_list(
        [SpatialId.common.object.point.Point(139.0, 35.0, 256)], 3857, 4326)

# convert_projected_point_list_to_point_listの呼び出し
## convert_projected_point_list_to_point_list(WGS84の座標をWebメルカトルの座標の座標に変換する)
SpatialId.shape.point.convert_projected_point_list_to_point_list(
        [SpatialId.common.object.point.Projected_Point(15473409.220265027, 4163881.144064293, 256)],
        3857, 4326)

# check_zoomの呼び出し
## check_zoom(指定の精度が有効範囲を超えていないかチェックをする)
SpatialId.shape.point.check_zoom(35)

# get_spatial_ids_on_polygonsの呼び出し
## 引数に使用するbarrier_triangles(チェック対象のポリゴン)の宣言
barrier_triangles = []
barrier_triangles.append(SpatialId.common.object.point.Triangle(
    SpatialId.common.object.point.Point(lon=153.192949805, lat=43.34880478, alt=13.55),
    SpatialId.common.object.point.Point(lon=153.192949805, lat=43.419462801, alt=18.55),
    SpatialId.common.object.point.Point(lon=153.192959805, lat=43.419462801, alt=19.5)))
barrier_triangles.append(SpatialId.common.object.point.Triangle(
    SpatialId.common.object.point.Point(lon=153.192949805, lat=43.34880478, alt=13.55),
    SpatialId.common.object.point.Point(lon=153.192949805, lat=43.419462801, alt=18.55),
    SpatialId.common.object.point.Point(lon=153.192959805, lat=43.419462801, alt=19.5)))
barrier_triangles.append(SpatialId.common.object.point.Triangle(
    SpatialId.common.object.point.Point(lon=153.192949805, lat=43.34880478, alt=13.55),
    SpatialId.common.object.point.Point(lon=153.192949805, lat=43.419462801, alt=18.55),
    SpatialId.common.object.point.Point(lon=153.192959805, lat=43.419462801, alt=19.5)))
barrier_triangles.append(SpatialId.common.object.point.Triangle(
    SpatialId.common.object.point.Point(lon=153.192949805, lat=43.34880478, alt=13.55),
    SpatialId.common.object.point.Point(lon=153.192949805, lat=43.419462801, alt=18.55),
    SpatialId.common.object.point.Point(lon=153.192959805, lat=43.419462801, alt=19.5)))

## 引数に使用するspace_triangles(除外対象のポリゴン)の宣言
space_triangles=[]
space_triangles.append(SpatialId.common.object.point.Triangle(
    SpatialId.common.object.point.Point(lon=137.0, lat=85.0, alt=49152.0),
    SpatialId.common.object.point.Point(lon=137.5, lat=84.94, alt=49152.),
    SpatialId.common.object.point.Point(lon=137.0, lat=84.94, alt=49152.)))
space_triangles.append(SpatialId.common.object.point.Triangle(
    SpatialId.common.object.point.Point(lon=137.0, lat=85.0, alt=49152.0),
    SpatialId.common.object.point.Point(lon=137.5, lat=84.94, alt=49152.0),
    SpatialId.common.object.point.Point(lon=137.0, lat=84.94, alt=49152.0)))
space_triangles.append(SpatialId.common.object.point.Triangle(
    SpatialId.common.object.point.Point(lon=137.0, lat=85.0, alt=49152.0),
    SpatialId.common.object.point.Point(lon=137.5, lat=84.94, alt=49152.0),
    SpatialId.common.object.point.Point(lon=137.0, lat=84.94, alt=49152.0)))
space_triangles.append(SpatialId.common.object.point.Triangle(
    SpatialId.common.object.point.Point(lon=137.0, lat=85.0, alt=49152.0),
    SpatialId.common.object.point.Point(lon=137.5, lat=84.94, alt=49152.0),
    SpatialId.common.object.point.Point(lon=137.0, lat=84.94, alt=49152.0)))

## get_spatial_ids_on_polygons(閉塞チェックあり、除外するポリゴンなし)
SpatialId.shape.polygons.get_spatial_ids_on_polygons(
        barrier_triangles,
        [],
        3,
        3,
        4326)

## get_spatial_ids_on_polygons(閉塞チェックあり、除外するポリゴンあり)
SpatialId.shape.polygons.get_spatial_ids_on_polygons(
        barrier_triangles,
        space_triangles,
        3,
        3,
        4326)

## get_spatial_ids_on_polygons(閉塞チェックなし、除外するポリゴンあり)
SpatialId.shape.polygons.get_spatial_ids_on_polygons(
        barrier_triangles,
        space_triangles,
        3,
        3,
        4326,
        False)


# cylinders.pyの呼び出し
## get_spatial_ids_on_cylinders(始点と終点が円柱状)
SpatialId.shape.cylinders.get_spatial_ids_on_cylinders(
        [SpatialId.common.object.point.Point(lon=153.192949805, lat=43.34880478, alt=13.55),
         SpatialId.common.object.point.Point(lon=153.192949805, lat=43.419462801, alt=18.55)],
        10,
        10,
        10,
        4326
        )
## get_spatial_ids_on_cylinders(始点と終点が球状)
SpatialId.shape.cylinders.get_spatial_ids_on_cylinders(
        [SpatialId.common.object.point.Point(lon=153.192949805, lat=43.34880478, alt=13.55),
         SpatialId.common.object.point.Point(lon=153.192949805, lat=43.419462801, alt=18.55)],
        10,
        10,
        10,
        4326,
        True
        )
## get_spatial_ids_on_cylinders(太さ0.4mで300m^3となる円柱)
SpatialId.shape.cylinders.get_spatial_ids_on_cylinders(
        [SpatialId.common.object.point.Point(lon=153.192949805, lat=43.34880478, alt=13.55),
         SpatialId.common.object.point.Point(lon=153.192949805, lat=43.34880478, alt=613.55)],
        0.4,
        26,
        26,
        4326
        )

#get_6spatial_ids_adjacent_to_facesの呼び出し
SpatialId.operated.shifting_spatial_id.get_6spatial_ids_adjacent_to_faces("20/20/20/20/20")

#get_8spatial_ids_around_horizontalの呼び出し
SpatialId.operated.shifting_spatial_id.get_8spatial_ids_around_horizontal("20/20/20/20/20")

#get_26spatial_ids_around_voxelの呼び出し
SpatialId.operated.shifting_spatial_id.get_26spatial_ids_around_voxel("20/20/20/20/20")

#get_shifting_spatial_idの呼び出し
## get_shifting_spatial_id(x,y,vを正の方向に移動)
SpatialId.operated.shifting_spatial_id.get_shifting_spatial_id("20/20/20/20/20", 2, 6, 9)
## get_shifting_spatial_id(xのみ負の方向に移動)
SpatialId.operated.shifting_spatial_id.get_shifting_spatial_id("20/20/20/20/20", -2, 6, 9)
## get_shifting_spatial_id(yのみ負の方向に移動)
SpatialId.operated.shifting_spatial_id.get_shifting_spatial_id("20/20/20/20/20", 2, -6, 9)
## get_shifting_spatial_id(vのみ負の方向に移動)
SpatialId.operated.shifting_spatial_id.get_shifting_spatial_id("20/20/20/20/20", 2, 6, -9)


## 引数に使用する集約関数の宣言
def _custom_payload_func(payload_dict: dict):
    agg_payload_dict = {}

    for key, value in payload_dict.items():
        # 集約関数発生の例外確認用
        if value[0][0] == 'Exception':
            raise Exception('集約関数内例外発生')

        # 集約
        agg_payload_dict[key] = value
        agg_payload_dict[key].append('agg_func add')

    return agg_payload_dict


# read_shapefileの呼び出し(shapefileは引数の位置に格納して下さい)
## read_shapefile(閉塞チェックあり)
SpatialId.io.shapefile.read_shapefile("./gis/sample/xxx.shp", "utf-8", 5, 5, 6677)
## read_shapefile(閉塞チェックなし)
SpatialId.io.shapefile.read_shapefile("./gis/sample/xxx.shp", "utf-8", 5, 5, 6677, False)

# read_shapefile_bulkの呼び出し
## read_shapefile_bulk(閉塞チェックあり)
SpatialId.io.shapefile.read_shapefile_bulk("./gis/sample/xxx.shp", "utf-8", 5, 5, 6677)
## read_shapefile_bulk(閉塞チェックなし)
SpatialId.io.shapefile.read_shapefile_bulk("./gis/sample/xxx.shp", "utf-8", 5, 5, 6677, False)

# read_shapefile_bulk_with_aggregationの呼び出し
## read_shapefile_bulk_with_aggregation(集約関数なし)
SpatialId.io.shapefile.read_shapefile_bulk_with_aggregation(
        "./gis/sample/xxx.shp", "utf-8", 5, 5, 6677)
## read_shapefile_bulk_with_aggregation(集約関数あり)
SpatialId.io.shapefile.read_shapefile_bulk_with_aggregation(
        "./gis/sample/xxx.shp", "utf-8", 5, 5, 6677, _custom_payload_func)
