dojo.provide("misc.URLQueryString");

dojo.declare("misc.URLQueryString", null, {
    /** Provides a simple API for accessing the query string parameters of the URL * */
    
    constructor: function(args) {
        /* any args provided will be considered additional query parameter values */
        if (args) {
            this.args = args;
        } else {
            this.args = {}
        }
        var uri = document.location.href;
        if (uri.indexOf("?") >= 0) {
            var uri_query = uri.substr(uri.indexOf("?") + 1, uri.length);
            var urlargs = dojo.queryToObject(uri_query);
            dojo.mixin(this.args, urlargs);
        }
    },

    get: function(key, default_value) {
        if (key in this.args) {
            return this.args[key];
        }
        return default_value;
    },
    
    getAll: function() {
        return this.args;
    }

});