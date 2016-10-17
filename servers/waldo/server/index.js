var crypto = require('crypto'); //For generating Auth Tokens
var mongoose = require('mongoose');
var express = require('express');
var passport = require('passport');
var LocalStrategy = require('passport-local').Strategy;

var app = express();
mongoose.connect('mongodb://localhost/test');

//Passport Authentication
app.use(passport.initialize());
app.use(passport.session());

//Dashboard "Front-End" Application
var dashboard_application = require("./app/main.js");
dashboard_application.make(express, app, passport, LocalStrategy);

// all environments
app.set('port', process.env.PORT || 3111);
var server = app.listen(app.get('port'));