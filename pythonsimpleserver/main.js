var url = "http://3xrp.localtunnel.com/android";

function keypressed(event) {
    var key = event.which;

    var x = 0, y = 0;
    if (key == "A".charCodeAt(0)) {
        x = -127;
    } else if (key == "W".charCodeAt(0)) {
        y = 127;
    } else if (key == "S".charCodeAt(0)) {
        y = -127;
    } else if (key == "D".charCodeAt(0)) {
        x = 127;
    } else {
        return;
    }

    getFile(url+"?x="+x+"&y="+y); 
}

function handleClick(num) {
    for (var i = 0; i < 5; i++) {
        var x = 0, y = 0;
        
        if (num == 0) {
            y = 127;
        } else if (num == 1) {
            y = -127;
        } else if (num == 2) {
            x = -127;
        } else if (num == 3) {
            x = 127;
        }

        getFile(url+"?x="+x+"&y="+y);
    }
}

function getFile(filename, postfunct, isAsync) {
    var ajaxRequest;

    if(window.XMLHttpRequest) 
        ajaxRequest = new XMLHttpRequest();
    else
        ajaxRequest = new ActiveXObject("Microsoft.XMLHTTP");

    ajaxRequest.open("GET", filename, true);
    ajaxRequest.send("random="+Math.random());
}
