$(function() {

    $('#side-menu').metisMenu();

});

//Loads the correct sidebar on window load,
//collapses the sidebar on window resize.
$(function() {
    $(window).bind("load resize", function() {
        console.log($(this).width())
        if ($(this).width() < 768) {
            $('div.sidebar-collapse').addClass('collapse')
        } else {
            $('div.sidebar-collapse').removeClass('collapse')
        }
    })

})



function createNewTab(href,name,image,li_class){
	function _getLastFolder(str){
		return str.match(/([^\/]*)\/*$/)[1]
	}

	var new_tab = $("#sidebar_tab").clone();
	new_tab.find("a").attr("href",href);
	new_tab.find(".tab_text").text(name);
	new_tab.find("i").attr('class',image);

	if(_getLastFolder(href)==_getLastFolder(window.location.pathname)){
		new_tab.find("li").attr('class','active');
	}
	

	return new_tab;
}

function loadSidebar(json_query){
	var sidebar_new = JSON.parse(json_query.replace(/&quot;/ig, '"'));
	$(".sidebar ul").empty();
	for (var i = 0; i < sidebar_new.length; i++) {
		var new_tab = createNewTab(sidebar_new[i]["href"],sidebar_new[i]["name"],sidebar_new[i]["image"]);
		$(".sidebar ul").append(new_tab.html());
	};

	$(".sidebar-back").remove();
	if (window.location.pathname.replace(/\//g,"")!="dashboard"){
		$(".sidebar ul").append(createNewTab("/dashboard/","Back","sidebar-back glyphicon glyphicon-arrow-left").html());
	}
}

function copyToClipboard(meintext) {  
if (window.clipboardData)   
     window.clipboardData.setData("Text", meintext);  
else if (window.netscape) {  
     netscape.security.PrivilegeManager.enablePrivilege('UniversalXPConnect');  
     var clip = Components.classes['@mozilla.org/widget/clipboard;1'].createInstance(Components.interfaces.nsIClipboard);  
     if (!clip)  
          return false;  
     var trans = Components.classes['@mozilla.org/widget/transferable;1'].createInstance(Components.interfaces.nsITransferable);  
     if (!trans)  
          return false;  
     trans.addDataFlavor('text/unicode');  
     var str = new Object();  
     var len = new Object();  
     var str = Components.classes["@mozilla.org/supports-string;1"].createInstance(Components.interfaces.nsISupportsString);  
     str.data=meintext;  
     trans.setTransferData("text/unicode",str,meintext.length*2);  
     var clipid=Components.interfaces.nsIClipboard;  
     if (!clipid)  
          return false;  
     clip.setData(trans,null,clipid.kGlobalClipboard);  
}  
     return false;  
}
