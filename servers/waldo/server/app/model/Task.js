var mongoose = require('mongoose'),
    Schema = mongoose.Schema,
    ObjectId = Schema.ObjectId;

var TaskSchema = new Schema({
    name: { type: String},
    client: {type:ObjectId},
});

module.exports = mongoose.model('Task', TaskSchema);