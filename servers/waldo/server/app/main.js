//Hosted from http://107.170.73.229:3111/

  app_info = {

    'name':'Waldo',
    'logo':'/images/logo.png'

  }

//Flash, to display login error messages
var flash = require('connect-flash');

//Express Middleware
var cookieParser = require('cookie-parser');
var cookieSession = require('cookie-session');
var bodyParser = require('body-parser');
var csrf = require('csurf');

//Move to Ember.js eventually
var hbs = require('hbs');
var fs = require('fs');
var mongoose = require('mongoose');

var login = require('./routes/login.js');

// Common partials here
hbs.registerPartials(__dirname + '/views/partials');


// Partials specific to app-side
hbs.registerPartial("sidebar",fs.readFileSync(__dirname + '/views/app/partials/sidebar.html','utf8'));
hbs.registerPartial("sidebar_tab",fs.readFileSync(__dirname + '/views/app/partials/sidebar_tab.html','utf8'));


var path = require('path'),
    appDir = path.dirname(require.main.filename);

//Load Models
var Client = require("./model/Client");
var Task = require("./model/Task");

hbs.registerHelper('partial', function (file,args,options) {
    args = this || {};
    var template = hbs.compile(fs.readFileSync(__dirname + '/views/' + file, 'utf8'));
    return new hbs.handlebars.SafeString(template(args));
});

function getClient(req,res){
  var user = req.user;

  // Redirect if user does not exist
  if (user==undefined){
    res.redirect("/login");
  }

  return user;
}

function render_template(req,res,template,path,data){
  render_parameters=data;
  render_parameters["body"]=path;

  if (req.header('X-PJAX')){
    //Pjax only wants to see the 'main content' of the page. [Pjax allows pages to be loaded much faster!]
    render(res,path,render_parameters);
  }else{

  	//Normal Rendering
    render(res,template,render_parameters);
  }
}

function render(res,path,data){
  render_parameters=data;
  render_parameters['app_info']=app_info;
  res.render(path,render_parameters);
}

function make(express, app, passport, LocalStrategy) {
      app.use(bodyParser());
      app.use(cookieParser('big secret'));
      app.use(cookieSession({
        secret : "big secret"
      }));

      app.use(csrf());
      app.use(function (req, res, next) {
        res.locals.csrftoken = req.csrfToken();
        next();
      });

      app.use(passport.initialize());
      app.use(passport.session());


      app.use(flash());
      app.set('view engine', 'html');
      app.set('views', __dirname + '/views');
      app.engine('html', hbs.__express);
      app.use(express.static(__dirname + '/public'));


  app.get('/', function(req, res){
      res.redirect("login");
  });

  app.get('/login', function(req, res){
      var error = req.flash('error');

      render_template(req,res,'app/templates/plain.html','app/partials/login.html',
        {'error':error,
         'title':'Login to '+app_info["name"],
         'login':true
        });
  });

  app.get('/register', function(req, res){

      var error = req.flash('error');

      render_template(req,res,'website/templates/plain.html','website/partials/login.html',
        {'error':error,
         'title':app_info["name"],
         'login':false
        });
  });

  app.get('/dashboard', function(req, res){

    var client = getClient(req,res);
    Task.find({ 'client': client._id },'' ,function (err, tasks) {
      render_template(req,res,'app/templates/main.html','app/partials/applications.html',
        {
          tasks: tasks,
          sidetabs: JSON.stringify([
                    {'name':'Status','href':'/dashboard/','image':'fa fa-dashboard fa-fw'},
                    {'name':'Vision','href':'/vision/','image':'fa fa-dashboard fa-fw'}]),
  	  js_load: '/js/subpages/applications.js'
        });
    });      

  });



  app.post('/dashboard/tasks', function(req, res){
      var inputted_name = req.body.application_name;
      var client = getClient(req,res);

      var newTask = new Task({
        name: inputted_name,
        client: client._id
      });

      // save user to database
      newTask.save(function(err) {
        res.redirect("/dashboard");
      });

  });

  app.get('/dashboard/tasks/:app_id/activity', function(req, res){
      var app_id = req.param("app_id"); //Mongo Idenfier of Task
      var client = getClient(req,res);
      Task.findOne({'_id': app_id },function(err,app){

        render_template(req,res,'app/templates/main.html','app/partials/activity.html',{
            sidetabs: create_sidebar_query(app_id),

            application: app,
            js_load: '/js/subpages/activity.js'

        });

      });

  });

  function create_sidebar_query(app_id){
    var sq = [
      {'name':'Activity','href':'/dashboard/tasks/'+app_id+'/activity/',
      'image':'glyphicon glyphicon-send'},
      {'name':'Logging','href':'/dashboard/tasks/'+app_id+'/logs/','image':
      'glyphicon glyphicon-send'},
    ];

    return JSON.stringify(sq)
  }

  app.post('/dashboard/tasks/:app_id/delete', function(req, res){
      var app_id = req.param("app_id"); //MongoId of task to be deleted
      var client = getClient(req,res);

      //Delete then re-direct to dashboard
      Task.find({_id:app_id},function(err,objs){
        var obj=objs[0];

        // Check whether this is the right user
        if (obj.client==client._id){
          obj.remove();
        }
        res.redirect("/dashboard");
      });

  });

  login.login_routes(app, passport, LocalStrategy);


}

exports.make=make;