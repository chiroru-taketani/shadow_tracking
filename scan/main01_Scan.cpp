// cd build
// make
// cd ..
// ./build/PointCloudToMesh
#include <fstream>
#include <iostream>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <string>
#include <vector>

int main()
{
    std::cout << "データの読み込みを開始します..." << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream file("scandata4.txt");
    std::string line;
    //調整が必要なパラメーター
    float z_max_limit = 600.0f; //奥行きがz_max_limitよりも小さいものを物体として認識
    int ignored_count = 0;

    while (std::getline(file, line))
    {
        float x, y, z;
        if (sscanf(line.c_str(), "%f,%f,%f", &x, &y, &z) == 3)
        {
            if (z < z_max_limit) //奥行きがz_max_limitよりも小さいものを物体として認識
            {
                cloud->push_back(pcl::PointXYZ(x, y, z));
            }
            else
            {
                ignored_count++;
            }
        }
    }

    if (cloud->empty())
    {
        std::cerr << "エラー: 読み込めた点が0個です。" << std::endl;
        return -1;
    }

    std::cout << "物体をグループ分けし、一番大きいものを探しています..." << std::endl;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cluster(new pcl::search::KdTree<pcl::PointXYZ>);
    tree_cluster->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(20.0);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(2500000);
    ec.setSearchMethod(tree_cluster);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    if (!cluster_indices.empty())
    {
        int max_size = 0;
        int largest_cluster_index = 0;
        for (int i = 0; i < cluster_indices.size(); ++i)
        {
            if (cluster_indices[i].indices.size() > max_size)
            {
                max_size = cluster_indices[i].indices.size();
                largest_cluster_index = i;
            }
        }
        for (const auto &idx : cluster_indices[largest_cluster_index].indices)
        {
            cloud_filtered->push_back((*cloud)[idx]);
        }
    }
    else
    {
        std::cerr << "\n【エラー】物体が見つかりませんでした。" << std::endl;
        return -1;
    }

    std::cout << "物体の縁（境界）を検出し、壁を作成しています..." << std::endl;

    // 1. 境界検出のために、まずは一時的に面の向きを計算します
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_temp;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_temp(new pcl::search::KdTree<pcl::PointXYZ>);
    ne_temp.setSearchMethod(tree_temp);
    ne_temp.setInputCloud(cloud_filtered);
    ne_temp.setKSearch(20);
    pcl::PointCloud<pcl::Normal>::Ptr normals_temp(new pcl::PointCloud<pcl::Normal>);
    ne_temp.compute(*normals_temp);

    // 2. 縁（境界）の点を探し出します
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud(cloud_filtered);
    est.setInputNormals(normals_temp);
    est.setRadiusSearch(15.0); // 縁を探す範囲
    est.setSearchMethod(tree_temp);
    pcl::PointCloud<pcl::Boundary> boundaries;
    est.compute(boundaries);

    // 3. 物体の一番奥のZ座標（最深部）を調べます
    float max_z_obj = -10000.0f;
    for (const auto &p : *cloud_filtered)
    {
        if (p.z > max_z_obj)
            max_z_obj = p.z;
    }


    float step_z = 3.0f;    // 壁を作る点の縦の間隔
    std::cout << "指定された床の高さ（Z = " << z_max_limit << "）まで壁を伸ばします..." << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud_filtered->size(); ++i)
    {
        if (boundaries[i].boundary_point > 0)
        {
            pcl::PointXYZ p = (*cloud_filtered)[i];

            // 床の高さ（floor_z）に到達するまで、点を奥へ追加していく
            while (p.z < z_max_limit)
            {
                p.z += step_z;

                // 🌟 追加: もし足した結果が床を通り過ぎてしまったら、床の高さでピッタリ止める
                if (p.z > z_max_limit)
                {
                    p.z = z_max_limit;
                }
                wall_cloud->push_back(p);
            }
        }
    }

    *cloud_filtered += *wall_cloud;
    std::cout << "壁の点群を追加しました（追加数: " << wall_cloud->size() << " 個）" << std::endl;
    // ==========================================================


    std::cout << "全体の面の向きを計算しています..." << std::endl;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered); // 壁も合体した状態のものを使用
    ne.setKSearch(20);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);

    std::cout << "3Dメッシュを生成しています..." << std::endl;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh mesh;

    gp3.setSearchRadius(30.0); // 探索範囲を少し広げる（15.0 → 20.0）
    gp3.setMu(3.5);
    gp3.setMaximumNearestNeighbors(500); // 探す候補の数
    gp3.setMaximumSurfaceAngle(M_PI );
    gp3.setMinimumAngle(M_PI / 180);    // 最小5度（細長い三角形もOKにする）
    gp3.setMaximumAngle(M_PI ); // 最大150度（歪んだ三角形もOKにする）
    gp3.setNormalConsistency(false);

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(mesh);

    pcl::io::savePLYFile("output_model.ply", mesh);
    std::cout << "成功しました！ 'output_model.ply' として保存されました。" << std::endl;

    return 0;
}
