var url = "http://3xrp.localtunnel.com/android";

function handleClick(num) {
    for (var i = 0; i < 5; i++) {
        if (num == 0) {
            getFile(url+"?x="+0+"&y="+127);
        } else if (num == 1) {
            getFile(url+"?x="+0+"&y="+(-127));
        } else if (num == 2) {
            getFile(url+"?x="+(-127)+"&y="+0);
        } else if (num == 3) {
            getFile(url+"?x="+(127)+"&y="+0);
        }
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
