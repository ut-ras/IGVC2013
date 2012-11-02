dojo.provide("cs148widgets.PythonExecutor");

dojo.require("cs148widgets.Utils");

dojo.declare("cs148widgets.PythonExecutor", null, {
    
    // Internal variables
    pythonscope: null,
    mainLoopTimeout: null,
    subscriptions: null,
    timers: null,
    
    constructor: function(args) {
    	// Initialise variables
        dojo.mixin(this, args);
        this.subscriptions = [];
        this.timers = [];
    },
    
    run: function(python) {
        // Stop any previous execution
        this.stop();
        
        // Configure the console
        Sk.configure({ 
            output: dojo.hitch(this, "onPrint"),
            read: this.builtinRead
        });
        
        // First, register this object's methods as the subscribe and publish python builtins
        Sk.builtins.publish = dojo.hitch(this, "publish");
        Sk.builtins.subscribe = dojo.hitch(this, "subscribe");
        Sk.builtins.repeat = dojo.hitch(this, "repeat");
        Sk.builtins.sleep = dojo.hitch(this, "sleep");
        
        var compiled;
        try {
            // Compile the python
            console.log("Compiling Python -> Narrative JavaScript");
            compiled = Sk.compile(python);
        } catch (e) {
            console.error("Exception compiling Python using Skulpt");
            throw e;
        }
        
        var code = compiled.code;
        if (code.indexOf("sleep->")!=-1) {
            try {
                // Compile with NJS if sleeps exist in code
                console.log("Compiling Narrative JavaScript -> JavaScript");
                code = new NjsCompiler().compile(code);
            } catch(e) {
                console.error("Exception compiling JavaScript using NarrativeJS");
            }
        } else {
            console.log("Skipping Narrative JavaScript -> JavaScript step")
        }
            
        try {
            // Execute
            console.log("Executing compiled JavaScript.");
            eval(code);
            eval("this.pythonscope = "+compiled.funcname+"()");
        } catch(e) {
            console.log("Exception running compiled JavaScript", e);
        }
        
    },
    
    builtinRead: function(x) {
        if (Sk.builtinFiles === undefined || Sk.builtinFiles["files"][x] === undefined)
            throw "File not found: '" + x + "'";
        return Sk.builtinFiles["files"][x];
    },
    
    stop: function() {
        window.clearTimeout(this.mainLoopTimeout);
        this.unsubscribeAll();
        this.clearAllTimers();
        this.pythonscope = null;
    },
    
    publish: function(topicobj, typeobj, messageobj) {
        var topic = Sk.ffi.remapToJs(topicobj);
        var type = Sk.ffi.remapToJs(typeobj);
        var message = Sk.ffi.remapToJs(messageobj);
        ros.publish(topic, type, dojo.toJson(message));
    },
    
    subscribe: function(topicobj, pythonCallback, rate) {
        if (!rate) {
            rate = 100; // Want to throttle a bit cos python is slooow
        }
        var topic = Sk.ffi.remapToJs(topicobj);
        var callback = dojo.hitch(this, "onMessage", topic, pythonCallback);
        ros.subscribe(topic, callback, rate);
        this.subscriptions.push({ "topic": topic, "callback": callback });
    },
    
    repeat: function(func, timeout) {
        if (!timeout) {
            timeout = 0;
        }
        var id = this.timers.length;
        this.timers.push(window.setTimeout(dojo.hitch(this, "repeatFunction", id, func, timeout), timeout));
    },
    
    repeatFunction: function(id, func, timeout) {
        Sk.misceval.callsim(func);
        window.clearTimeout(this.timers[id]);
        this.timers[id] = window.setTimeout(dojo.hitch(this, "repeatFunction", id, func, timeout), timeout);        
    },
    
    sleep: function(duration) {
        // Placeholder so that parser allows 'sleep' calls.  Modified skulpt compiler should replace any
        // any calls to "sleep(x)" with just "sleep->(x)", for further parsing by njs.  
    	// See compile.js line 311
    },
    
    unsubscribeAll: function() {
        for (var i = 0; i < this.subscriptions.length; i++) {
            var subscription = this.subscriptions[i];
            ros.unsubscribe(subscription.topic, subscription.callback);
        }
        this.subscriptions = [];
    },
    
    clearAllTimers: function() {
        for (var i = 0; i < this.timers.length; i++) {
            window.clearTimeout(this.timers[i]);
        }
        this.timers = [];
    },
    
    onMessage: function(topic, callback, msg) {
        // First turn the msg Json into an object
        var pythonObj = Sk.ffi.remapToPy(msg);
        
        // Then call the callback with the object
        try {
            Sk.misceval.callsim(callback, pythonObj);
        } catch (e) {
            console.error("Python callback for topic "+topic+" threw exception", e);
        }
    },
    
    // Override me. Called with stdout
    onPrint: function(str) {}
    
});