if ($('#request_origins').length>0){
	$('#request_origins').tagsInput({width:'auto',height:'auto',defaultText:''});
	$(".tagsinput").addClass("form-control");  
}
$('#request_origins').importTags('localhost,127.0.0.1');

$( "#security_save" ).click(function() {
	$("#security_form").submit();
})


$("#generate_new_key").click(function(){
	
	return false;
});