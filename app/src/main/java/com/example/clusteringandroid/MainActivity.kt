package com.example.clusteringandroid

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Environment
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.material.CircularProgressIndicator
import androidx.compose.material.MaterialTheme
import androidx.compose.material.Surface
import androidx.compose.material.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import com.example.clusteringandroid.ui.theme.ClusteringAndroidTheme
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import java.nio.file.Files
import java.nio.file.Paths
import java.text.SimpleDateFormat
import java.util.*
import kotlin.io.path.name
import kotlin.streams.toList

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            ClusteringAndroidTheme {
                // A surface container using the 'background' color from the theme
                Surface(modifier = Modifier.fillMaxSize(), color = MaterialTheme.colors.background) {
                    MainView("image")
                }
            }
        }
        System.loadLibrary("clustering_android")
    }
}

@Composable
fun MainView(type: String = "pc") {
    val context = LocalContext.current
    var done by remember { mutableStateOf(false) }
    var statusText by remember { mutableStateOf("") }
    var launched by remember {
        mutableStateOf(false)
    }

    if (ContextCompat.checkSelfPermission(context, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
        Log.i(MainActivity::class.simpleName, "No permission");
        Text(text = "Done", fontSize = 50.sp)
        val launcher = rememberLauncherForActivityResult(
            ActivityResultContracts.RequestPermission()
        ) {
            if (it) Log.i(MainActivity::class.simpleName, "Granted");

        }
        SideEffect {
            launcher.launch(Manifest.permission.WRITE_EXTERNAL_STORAGE)
        }
    }



    SideEffect {
        if (!launched) {
            launched = true
            GlobalScope.launch {
                val date = Calendar.getInstance().time;
                val dateFormat = SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");
                val formatedDate = dateFormat.format(date)

                val folderName = when(type){
                    "pc" -> "Stanford3dDatasetCSV"
                    "image" -> "original-images"
                    else -> throw IllegalArgumentException("Argument $type is not correct")
                }
                val baseFolder = Paths.get(Environment.getExternalStorageDirectory().path, "Documents", folderName)
                val resultsFile = Paths.get(Environment.getExternalStorageDirectory().path, "Documents", "results_$formatedDate.json")

                val fileList = Files.list(baseFolder).map { it.name }.toList()
                for ((i, image) in fileList.withIndex()) {
                    statusText = "${image.split(".")[0]} ${i + 1}/${fileList.size}"
                    if (type == "pc")
                        clusterPC(baseFolder.toString(), resultsFile.toString(), image, intArrayOf(2, 4, 6), intArrayOf(1, 2, 4, 100, 1000), 1, false, true)
                    else
                        clusterImage(baseFolder.toString(), resultsFile.toString(), image, intArrayOf(1000,2000,4000), intArrayOf(1, 2, 4, 100, 1000), 1, false, false)

                }
                dumpJSON(resultsFile.toString())
                done = true
            }
        }
    }

    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        if (!done) {
            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                CircularProgressIndicator(modifier = Modifier.size(100.dp))
                Spacer(modifier = Modifier.size(10.dp))
                Text(text = statusText, fontSize = 20.sp)
            }
        } else
            Text(text = "Done", fontSize = 50.sp)
    }
}

external fun clusterImage(basePath: String, outputPath: String, imageName: String, clusteringFactors: IntArray, perforationFactors: IntArray, repeats: Int, combs: Boolean, offset_start: Boolean)
external fun clusterPC(basePath: String, outputPath: String, pcName: String, clusteringFactors: IntArray, perforationFactors: IntArray, repeats: Int, combs: Boolean, offset_start: Boolean)
external fun dumpJSON(outputPath: String)