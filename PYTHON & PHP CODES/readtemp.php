<?php
if (isset($_GET["q"]) && isset($_GET["appid"])) {
    $fp = fopen("log.txt", "a");
    fprintf($fp, "%s: Asked for the weather in %s with appkey: %s\r\n", date("Y-m-d H:i:s"), $_GET['q'], $_GET['appid']);
    fclose($fp);

    // Read data from files
    $temp  = file_exists("temp.txt")  ? trim(file_get_contents("temp.txt"))  : "N/A";
    $press = file_exists("press.txt") ? trim(file_get_contents("press.txt")) : "N/A";
    $hum   = file_exists("hum.txt")   ? trim(file_get_contents("hum.txt"))   : "N/A";

    // Return all as JSON
    header('Content-Type: application/json');
    echo json_encode([
        "temp"  => $temp,
        "press" => $press,
        "hum"   => $hum
    ]);
} else {
    header("HTTP/1.1 400 Bad Request");
    echo json_encode(["error" => "Missing 'q' or 'appid'"]);
}
?>
