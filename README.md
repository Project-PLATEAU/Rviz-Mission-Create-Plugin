#  FY2022 Project PLATEAU UC22-024「3D都市モデルとBIMを活用したモビリティ自律運行システム」の成果物(Rviz-Mission-Create-Plugin)
# Rviz-Mission-Create-Plugin
![rviz](https://user-images.githubusercontent.com/79615787/227794142-3398e974-2feb-4d22-9627-8ed2c090678c.jpg)

## 1. 概要
「Rviz-Mission-Create-Plugin」は、ROSの標準ビューワソフトであるRviz上でロボットに移動指示・方向転換指示などを与えるために必要な機能を提供するプラグインとそれをRviz上で表示するための設定ファイルのパッケージです。


### ユースケースの概要
都市部における建設工事では資材運搬等による交通渋滞が課題となります。 そこで、自律運航可能なドローンや無人搬送車両（AGV）の活用による解決が期待されている一方で、自動運転モビリティの運航に必要な地図情報として民間事業者が提供する3Dマップを利用せざるを得ず、精度担保やデータ連携、カバレッジ等の点での課題もあります。 本業務では、LiDARやGPS等のセンサーと3D都市モデルを利用した自己位置測位を組み合わせた運航システムを開発しました。

### 開発システムの概要
本業務では、屋外は3D都市モデル（建築物モデルLOD2-3）を、屋内はBIMモデルを活用してLiDARとGNSS（衛星測位システム）による自己位置推定を行うドローン自律飛行システムを、ロボット・アプリケーションの作成支援ライブラリ・ツール群であるROSを利用して構築しました。  
「Rviz-Mission-Create-Plugin」は、ドローンに対してGUI上でウェイポイント設定及び方向転換等のアクション指示を行うことができるRvizプラグインです。  
詳細は[技術実証レポート](https://www.mlit.go.jp/plateau/file/libraries/doc/plateau_tech_doc_0046_1_ver01.pdf)を参照してください。
<img src="Documentation/resources/Usage/final_result.jpg" width="80%">

## 3．利用手順

 インストール方法・使い方は[こちら](https://project-plateau.github.io/PLATEAU-UC22-024-Rviz-Mission-Create-Plugin/)

## ライセンス
* 本ツールは、令和4年度 民間ユースケース開発　（UC22-024）「3D都市モデルとBIMを活用したモビリティ自律運行システム」の中で[Sensyn Robotics](https://www.sensyn-robotics.com/)が開発したものです。ソースコードおよび関連ドキュメントの著作権は国土交通省に帰属します。
* 本ドキュメントは[Project PLATEAUのサイトポリシー](https://www.mlit.go.jp/plateau/site-policy/)（CCBY4.0および政府標準利用規約2.0）に従い提供されています。

## 注意事項
* 本レポジトリは参考資料として提供しているものです。動作保証は行っておりません。
* 予告なく変更・削除する可能性があります。
* 本レポジトリの利用により生じた損失及び損害等について、国土交通省はいかなる責任も負わないものとします。

## 参考資料
* 3D都市モデルとBIMを活用したモビリティ自律運行システム技術検証レポート: https://www.mlit.go.jp/plateau/libraries/technical-reports/
* PLATEAU Webサイト Use caseページ「3D都市モデルとBIMを活用したモビリティ自律運行システム」: https://www.mlit.go.jp/plateau/use-case/uc22-024/
* 利用しているライブラリなどへのリンク
  * [pcl_ros](http://wiki.ros.org/pcl_ros) : point cloud libraryの基本機能を有したRosパッケージ
  * [octomap](http://wiki.ros.org/octomap) : octomap形式のデータを扱うためのRosパッケージ
  * [jsk_visualization](http://wiki.ros.org/jsk_visualization) : Rviz UI上にビジュアライズ可能なツール群を有するRosパッケージ

