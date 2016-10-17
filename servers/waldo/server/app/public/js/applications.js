function deleteApplication(id){	
	$('#deleteModal').find("form").attr("action","/dashboard/apps/"+id+"/delete");
	$('#deleteModal').modal('show');
}
