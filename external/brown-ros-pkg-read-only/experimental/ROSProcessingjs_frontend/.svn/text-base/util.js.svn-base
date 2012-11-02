
// dummpy function
function nop() {}

// returns object info as a string
function concatObject(obj) {
  var str='';

  if(typeof(obj) == "string")
    return obj;

  for(prop in obj) {
    str += prop + " : ";
    
    if (typeof(obj[prop]) == "object") 
      str += concatObject(obj[prop]) + "\n";
    else
      str += "[ " + obj[prop] + "] ";
  }
  return(str);
}

function stopRKey(evt) {
  var evt = (evt) ? evt:((event) ? event: null);
  var node = (evt.target) ? evt.target : ((evt.srcElement) ? evt.srcElement : null);
  if((evt.keyCode == 13) && (node.type == "text")) { return false; }
}

document.onkeypress = stopRKey;
