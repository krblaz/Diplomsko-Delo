#include <jni.h>
#include <vector>
#include <string>
#include "clustering/clustering.h"

auto jarrayToArray(JNIEnv *env, jintArray jarray) {
    int size = env->GetArrayLength(jarray);
    auto elements = env->GetIntArrayElements(jarray, nullptr);
    std::vector<int> array(size);
    for (int i = 0; i < size; ++i) {
        array[i] = elements[i];
    }
    return array;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_clusteringandroid_MainActivityKt_clusterImage(JNIEnv *env, jclass clazz, jstring base_path_c, jstring output_path_c, jstring image_name_c, jintArray clustering_factors_c, jintArray perforation_factors_c, jint repeats, jboolean combs, jboolean offset_start) {
    auto base_path = std::string(env->GetStringUTFChars(base_path_c, nullptr));
    auto output_path = std::string(env->GetStringUTFChars(output_path_c, nullptr));
    auto image_name = std::string(env->GetStringUTFChars(image_name_c, nullptr));

    auto cluster_f = jarrayToArray(env, clustering_factors_c);
    auto perf_f = jarrayToArray(env, perforation_factors_c);

    clusterImage(base_path, output_path, image_name, cluster_f, perf_f, repeats, combs, offset_start);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_clusteringandroid_MainActivityKt_clusterPC(JNIEnv *env, jclass clazz, jstring base_path_c, jstring output_path_c, jstring pc_name_c, jintArray clustering_factors_c, jintArray perforation_factors_c, jint repeats, jboolean combs, jboolean offset_start) {
    auto base_path = std::string(env->GetStringUTFChars(base_path_c, nullptr));
    auto output_path = std::string(env->GetStringUTFChars(output_path_c, nullptr));
    auto pc_name = std::string(env->GetStringUTFChars(pc_name_c, nullptr));

    auto cluster_f = jarrayToArray(env, clustering_factors_c);
    auto perf_f = jarrayToArray(env, perforation_factors_c);

    clusterPC(base_path, output_path, pc_name, cluster_f, perf_f, repeats, combs, offset_start);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_clusteringandroid_MainActivityKt_dumpJSON(JNIEnv *env, jclass clazz, jstring output_path_c) {
    auto output_path = std::string(env->GetStringUTFChars(output_path_c, nullptr));
    std::ofstream outstream(output_path);
    outstream << final_results.dump(2);
}