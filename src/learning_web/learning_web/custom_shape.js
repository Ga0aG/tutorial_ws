// 地图，与ROS2D相比可以
// 自选不确定区域的颜色
OccupancyGrid = function(options) {
    options = options || {};
    var color_ = options.color_;
    var message = options.message;
    // internal drawing canvas
    var canvas = document.createElement('canvas');
    var context = canvas.getContext('2d');
    // console.log(canvas.tagName);
    // save the metadata we need
    this.pose = new ROSLIB.Pose({
        position : message.info.origin.position,
        orientation : message.info.origin.orientation
    });
    // set the size
    this.width = message.info.width;
    this.height = message.info.height;
    this.resolution = message.info.resolution;
    console.log(this.width, this.height);
    canvas.width = this.width;
    canvas.height = this.height;
    var imageData = context.createImageData(this.width, this.height);
    for ( var row = 0; row < this.height; row++) {
        for ( var col = 0; col < this.width; col++) {
        // determine the index into the map data
        var mapI = col + ((this.height - row - 1) * this.width);
        // determine the value
        var data = message.data[mapI];
        var val;
        // determine the index into the image data array
        var i = (col + (row * this.width)) * 4;
        // if (data > 100) {
        //     val = 0;
        //     // r
        //     imageData.data[i] = val;
        //     // g
        //     imageData.data[++i] = val;
        //     // b
        //     imageData.data[++i] = val;
        // } else 
        if (data === 0) {
            val = 255;
            // r
            imageData.data[i] = val;
            // g
            imageData.data[++i] = val;
            // b
            imageData.data[++i] = val;
        } else {
            // val = 127;
            // r
            imageData.data[i] = color_.r*(100-data)/100;
            // g
            imageData.data[++i] = color_.g*(100-data)/100;
            // b
            imageData.data[++i] = color_.b*(100-data)/100;
        }
        // imageData.data[i] = 255 - data;
        // imageData.data[++i] = 255 - data;
        // imageData.data[++i] = 255 - data;
        // a
        imageData.data[++i] = 255;
        }
    }
    this.imageData = imageData;
    this.message = message;
    context.putImageData(imageData, 0, 0);
    // create the bitmap
    createjs.Bitmap.call(this, canvas);
    // change Y direction
    this.y = -this.height * message.info.resolution;
    
    // scale the image
    this.scaleX = message.info.resolution;
    this.scaleY = message.info.resolution;
    this.width *= this.scaleX;
    this.height *= this.scaleY;
    // set the pose
    this.x += this.pose.position.x;
    this.y -= this.pose.position.y;
  };
OccupancyGrid.prototype.__proto__ = createjs.Bitmap.prototype;

OccupancyGridClient = function(options) {
    var that = this;
    options = options || {};
    var ros = options.ros;
    var topic = options.topic || '/map';
    this.continuous = options.continuous;
    this.rootObject = options.rootObject || new createjs.Container();
    // current grid that is displayed
    // create an empty shape to start with, so that the order remains correct.
    this.currentGrid = new createjs.Shape();
    this.rootObject.addChild(this.currentGrid);
    // work-around for a bug in easeljs -- needs a second object to render correctly
    this.rootObject.addChild(new ROS2D.Grid({size:1}));
    // subscribe to the topic
    var rosTopic = new ROSLIB.Topic({
        ros : ros,
        name : topic,
        messageType : 'nav_msgs/OccupancyGrid',
        compression : 'png'
    });
    rosTopic.subscribe(function(message) {
        // check for an old map
        var index = null;
        if (that.currentGrid) {
            index = that.rootObject.getChildIndex(that.currentGrid);
            that.rootObject.removeChild(that.currentGrid);
        }
        that.currentGrid = new OccupancyGrid({
            message : message,
            color_  : options.color_
        });
        if (index !== null) {
            that.rootObject.addChildAt(that.currentGrid, index);
        }
        else {
            that.rootObject.addChild(that.currentGrid);
        }
            that.emit('change');
        // check if we should unsubscribe
        if (!that.continuous) {
            rosTopic.unsubscribe();
        }
    });
};
OccupancyGridClient.prototype.__proto__ = EventEmitter2.prototype;

// 激光
ScanShape = function(options) {
    options = options || {};
    var scan = options.scan;
    this.strokeSize = options.strokeSize || 3;
    this.strokeColor = options.strokeColor || createjs.Graphics.getRGB(0, 0, 0);
    
    // draw the line
    this.graphics = new createjs.Graphics();
    
    if (scan !== null && typeof scan !== 'undefined') {
        this.graphics.setStrokeStyle(this.strokeSize);
        this.graphics.beginStroke(this.strokeColor);
        for (var i=0; i<scan.poses.length; ++i) {
            this.graphics.drawRect(scan.poses[i].pose.position.x / this.scaleX -0.03/this.scaleX,scan.poses[i].pose.position.y / -this.scaleY-0.03/this.scaleX, 0.06/this.scaleX, 0.06/this.scaleX);
        }
        this.graphics.endStroke();
    }
    
    // create the shape
    createjs.Shape.call(this, this.graphics);
};

ScanShape.prototype.setScan = function(scan) {
    this.graphics.clear();
    if (scan !== null && typeof scan !== 'undefined') {
        this.graphics.setStrokeStyle(this.strokeSize);
        this.graphics.beginStroke(this.strokeColor);
        for (var i=0; i<scan.poses.length; ++i) {
            this.graphics.drawRect(scan.poses[i].pose.position.x / this.scaleX -0.03/this.scaleX,scan.poses[i].pose.position.y / -this.scaleY-0.03/this.scaleX, 0.06/this.scaleX, 0.06/this.scaleX);
        }
        this.graphics.endStroke();
    }
};
ScanShape.prototype.__proto__ = createjs.Shape.prototype;

// 虚拟墙
VirtualWallShape = function() {
    this.strokeSize = 0.05;
    this.strokeColor = createjs.Graphics.getRGB(0, 0, 255, 0.5);
    this.graphics = new createjs.Graphics();
    createjs.Shape.call(this, this.graphics);
};
VirtualWallShape.prototype.setWall = function(wall) {
    this.graphics.clear();
    this.graphics.setStrokeStyle(this.strokeSize);
    this.graphics.beginStroke(this.strokeColor);
    this.graphics.beginStroke(this.strokeColor);
    this.graphics.drawRect( wall[0].x/this.scaleX - 0.03/this.scaleX, 
                            -wall[0].y/this.scaleY - 0.03/this.scaleY, 0.06/this.scaleX, 0.06/this.scaleX);
    
    this.graphics.moveTo(wall[0].x/this.scaleX, -wall[0].y/this.scaleY);
    for(var i = 1; i < wall.length; i++){
        this.graphics.lineTo(wall[i].x/this.scaleX, -wall[i].y/this.scaleY);
        this.graphics.drawRect( wall[i].x/this.scaleX - 0.03/this.scaleX, 
                                -wall[i].y/this.scaleY - 0.03/this.scaleY, 0.06/this.scaleX, 0.06/this.scaleX);
    }
    this.graphics.endStroke();
}
VirtualWallShape.prototype.__proto__ = createjs.Shape.prototype;

function DrawPillars() {
    createjs.Shape.call(this);//继承Shap类
    this.graphics.beginFill("#ff0000").drawRect(0, 0, 50, 50);
    this.setBounds(0,0,50,50);//设置矩形的边界属性，这样可以获得width和height属性
}
DrawPillars.prototype = new createjs.Shape();//获得原型方法

