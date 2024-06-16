
"use strict";

let BoundingBox = require('./BoundingBox.js');
let ObjectCount = require('./ObjectCount.js');
let BoundingBoxes = require('./BoundingBoxes.js');
let CheckForObjectsAction = require('./CheckForObjectsAction.js');
let CheckForObjectsActionResult = require('./CheckForObjectsActionResult.js');
let CheckForObjectsFeedback = require('./CheckForObjectsFeedback.js');
let CheckForObjectsActionFeedback = require('./CheckForObjectsActionFeedback.js');
let CheckForObjectsResult = require('./CheckForObjectsResult.js');
let CheckForObjectsGoal = require('./CheckForObjectsGoal.js');
let CheckForObjectsActionGoal = require('./CheckForObjectsActionGoal.js');

module.exports = {
  BoundingBox: BoundingBox,
  ObjectCount: ObjectCount,
  BoundingBoxes: BoundingBoxes,
  CheckForObjectsAction: CheckForObjectsAction,
  CheckForObjectsActionResult: CheckForObjectsActionResult,
  CheckForObjectsFeedback: CheckForObjectsFeedback,
  CheckForObjectsActionFeedback: CheckForObjectsActionFeedback,
  CheckForObjectsResult: CheckForObjectsResult,
  CheckForObjectsGoal: CheckForObjectsGoal,
  CheckForObjectsActionGoal: CheckForObjectsActionGoal,
};
