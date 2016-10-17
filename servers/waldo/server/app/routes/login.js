var Client = require("../model/Client");

function login_routes(app, passport, LocalStrategy){
  passport.serializeUser(function(user, done) {
    done(null, user);
  });

  passport.deserializeUser(function(user, done) {
    done(null, user);
  });

  //Strategy for logging in
  passport.use(new LocalStrategy({
      usernameField: 'email',
      passwordField: 'password'
    },
    function(email, password, done) {
     
      process.nextTick(function () {
        Client.getAuthenticated(email, password, function(err, user, reason) {
            if (err) { return done(null, false, { message:
              'Invalid email or password.' }); }

            // otherwise we can determine why we failed
            var reasons = Client.failedLogin;
            switch (reason) {
                case reasons.NOT_FOUND:
                    return done(null, false, { message:
                      'User does not exist yet, were you trying to sign up?' });
                    break;
                case reasons.PASSWORD_INCORRECT:
                    // note: these cases are usually treated the same - don't tell
                    // the user *why* the login failed, only that it did
                    return done(null, false, { message:
                      'Invalid email or password.' });
                    break;
                case reasons.MAX_ATTEMPTS:
                    // send email or otherwise notify user that account is
                    // temporarily locked
                    return done(null, false, { message: 
                      'Your account has been locked out. Please contact us.' });
                    break;
            }

            return done(null, user);
          });
      });
    }
  ));


  app.get('/loginFailure' , function(req, res, next){
    res.send('Failure to authenticate');
  });

  app.get('/loginSuccess' , function(req, res, next){
    res.send('Successfully authenticated');
  });

  app.post('/login',
    passport.authenticate('local', {
      successRedirect: '/dashboard',
      failureRedirect: '/login',
      failureFlash: true
    }));

  app.post('/register',function(req, res){

    // create a new user
    // email is already set to unique
    var newClient = new Client({
        first_name: req.body.first_name,
        email: req.body.email,
        password: req.body.password,
    });

    // save user to database
    newClient.save(function(err) {
      if (err){
        req.flash('error','Could not create new account, please contact us.');
        res.redirect("/register");
      }else{
        //Successful Login
        res.redirect("/dashboard");
      }
    });

  });
}
exports.login_routes=login_routes;