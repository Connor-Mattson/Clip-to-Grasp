"""
Given the 3D AABB of an object in world space, determine the 2D bounding box in the image.
"""

import itertools
import cv2
from src.perception.projection import project_point

class AABBCropper:
    def __init__(self, sim, robot, image, objects):
        self.sim = sim
        self.robot = robot
        self.image = image
        self.objects = objects
        self.cropped_images = []
        self.twod_boxes = []

        self.w, self.h = self.image.shape[:2]

    def get_object_aabb(self, obj_id):
        box_min, box_max = self.sim.get_object_aabb(obj_id)
        points = list(itertools.product(
            [box_min[0], box_max[0]],
            [box_min[1], box_max[1]],
            [box_min[2], box_max[2]],
        ))  # List of 8 (x, y, z)
        return points

    def project_to_camera(self, point):
        return project_point(point, self.robot.view_matrix, self.robot.projection_matrix, self.w, self.h)

    def project_aabb(self, obj_id):
        points = self.get_object_aabb(obj_id)
        xs, ys = zip(*[self.project_to_camera(point) for point in points])
        x = max(min(xs), 0), min(max(xs), self.w-1)
        y = max(min(ys), 0), min(max(ys), self.h-1)
        return x, y

    def draw_bounding_boxes(self,image, boxes, labels=None, color=(0, 255, 0), thickness=2):
        """
        Draws bounding boxes on a copy of the input image.
        """
        img_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR).copy()
        for i, (x1, y1, x2, y2) in enumerate(boxes):
            cv2.rectangle(img_bgr, (x1, y1), (x2, y2), color, thickness)
            if labels and i < len(labels):
                cv2.putText(img_bgr, labels[i], (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, 1, cv2.LINE_AA)
        return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)


    def crop_object(self, obj_id):
        x, y = self.project_aabb(obj_id)
        return self.image[y[0]:y[1], x[0]:x[1]], (x[0], y[0], x[1], y[1])

    def crop_all_objects(self):
        cropped_images = []
        twod_boxes = []
        for obj in self.objects:
            cropped_image, box = self.crop_object(obj.oid)
            cropped_images.append(cropped_image)
            twod_boxes.append(box)
        self.cropped_images = cropped_images
        self.twod_boxes = twod_boxes

        boxes_img = self.draw_bounding_boxes(self.image, self.twod_boxes)
        return cropped_images, boxes_img
