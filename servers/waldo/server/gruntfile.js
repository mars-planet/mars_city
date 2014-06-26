'use strict';
var path = require('path');
var mountFolder = function (connect, dir) {
    return connect.static(path.resolve(dir));
};

module.exports = function(grunt) {

    // Automatically load grunt tasks.
    require('load-grunt-tasks')(grunt);

    // Project configuration.
    grunt.initConfig({
        pkg: grunt.file.readJSON('package.json'),

        watch: {
            server: {
                files: [
                    'app/{,*/}*.js',
                    'app/**/*.*'
                ],
                tasks: [
                    'shell:launch_server:kill',
                    'shell:launch_server'
                ],
                options: { nospawn: true }
            },
        },

        shell: {
            launch_server: {
                options: {
                    stderr: true,
                    stdout: true,
                    async: true,
                    failOnError: false
                },
                command: 'killall node;node index.js'
            },
        }
    });

    // Default task(s).
    grunt.registerTask('serve', [
        //'jshint', // FIXME: Include once all files have been Linted.
        //'jsdoc', // FIXME
        'shell:launch_server',
        'watch'
    ]);

};
