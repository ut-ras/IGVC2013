<?php
set_include_path(get_include_path() . ':/home/tjay/public_html/test/ZendGdata-1.10.7/library');

require_once 'Zend/Loader.php';
Zend_Loader::loadClass('Zend_Gdata');
Zend_Loader::loadClass('Zend_Gdata_AuthSub');
Zend_Loader::loadClass('Zend_Gdata_ClientLogin');
Zend_Loader::loadClass('Zend_Gdata_Calendar');

//credentials, calendarURL
$user = 'brownroboticsrobot@gmail.com';
$pass = 's37iouSly';
//don't forget to change for DST
$offsetSeconds = -4*60*60;
$offset = '-04:00'; 

header('Content-type: text/javascript');

$jsonp = $_GET['jsonp'];
if ($jsonp == null) $jsonp = 'invoke';

$q = $_GET['q'];
if ($q == null) {
	$q = 'rosjs-session';
} else {
	$q = urldecode($q);	
}

$client = Zend_Gdata_ClientLogin::getHttpClient($user,$pass,Zend_Gdata_Calendar::AUTH_SERVICE_NAME);
$cal = new Zend_Gdata_Calendar($client);

$query = $cal->newEventQuery();
$query->setUser('brownrobotics@gmail.com');
$query->setVisibility('private');
$query->setProjection('full');
$now = time();
echo "//" . date('Y-m-d\TG:i:s',$now) . $offset . "\n";
$soon = time() + 60*5;
$query->setStartMin(date('Y-m-d\TH:i:s',$now) . $offset);
$query->setStartMax(date('Y-m-d\TH:i:s',$soon) . $offset);
$query->setQuery($q);

$feed = $cal->getCalendarEventFeed($query);
$guests = array();
foreach ($feed as $event) {
	if (strpos($event->title->text,$q) != 0) continue;

	//we assume the same time zone
	$time = $event->when[0];
	$year = substr($time, 44, 4);
	$month = substr($time,49,2);
	$day = substr($time,52,2);
	$hour = substr($time,55,2);
	$min = substr($time,58,2);
	$sec = substr($time,61,2);

	$ends = strtotime($month . '/' . $day . '/' . $year);
	$ends += $hour*60*60;
	$ends += $min*60;
	$ends += $sec;

	$left = $ends - $now;

	foreach ($event->who as $who) {
		$guests["$who"] = $left;
	}
}


echo $jsonp . "([";
foreach (array_keys($guests) as $guest) {
	echo "[\"" . $guest . "\"," . $guests[$guest] . "],";
}
echo "]);";

?>
