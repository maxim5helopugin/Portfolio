from training import Trainer
from detection import Detector, load_features


def main():
    """Create a trained feature set.
        1. Extract features from a cropped image set
        2. Evaluate features on labelled training data
        3. Remove features with high false positive rate
        4. Evaluate features again on same data
    """

    trainer = Trainer()
    detector = Detector(750)
    trainer.set_detector(detector)
    # feat = load_features('data/gazebo1_932.dat')
    # trainer.set_features(feat)

    trainer.load_data('train/gazebo/pos_info_PreyEmptyWorld.dat')
    trainer.load_data('train/gazebo/pos_info_PurplePrey1.dat')
    trainer.load_data('train/gazebo/pos_info_PurplePrey2.dat')
    trainer.load_data('train/gazebo/sim2.dat')

    # trainer.load_data('train/quadbox/white_courtyard1.dat')
    # trainer.load_data('train/quadbox/white_courtyard2.dat')
    # trainer.load_data('train/quadbox/black_drone_court.dat')
    # trainer.load_data('train/quadbox/pos_info.txt')

    # trainer.load_data('train/idea2/pos_info_red-inclass.dat')
    # trainer.load_data('train/idea2/red2m.dat')

    # trainer.load_data('train/idea2/courtyard323.dat')
    # trainer.load_data('train/idea2/orange10am.dat')
    # trainer.load_data('train/idea2/orange7_30am.dat')
    # trainer.load_data('train/pos_info_Courtyard_multi.dat')
    # trainer.load_data('train/idea2/orange9am.dat')

    # trainer.subsample_data(2)
    trainer.train_and_test(.8)
    trainer.evaluate(1)
    trainer.feature_selection()
    # trainer.evaluate(subsample=0.4)
    # trainer.feature_selection()
    # trainer.evaluate(subsample=0.7)
    trainer.save_features('sim2')
    return

    trainer.train_and_test(show=False)
    trainer.feature_selection()
    trainer.evaluate(show=False)
    trainer.save_features('ir_2')

    return
    # train_images = 'train/gazebo/set1'
    # data = 'train/quadbox/pos_info.txt'
    #
    # train_set = load_image_folder(train_images)
    # # trainer.train_images(images=train_set, show=1)
    #
    # trainer.load_data(data)
    # trainer.train_and_test(show=False)
    # trainer.feature_selection()
    # trainer.evaluate(show=True)
    # trainer.save_features('qb1')
    # return
    #
    # trainer.load_data(data)
    # detector.set_max_features(700)
    # trainer.evaluate(1)


    # detector = Detector(35)
    #
    # trainer = Trainer()
    # trainer.set_detector(detector)
    # training_set1 = load_image_folder('train/idea2/orange2')
    # training_set2 = load_image_folder('train/idea2/orange3')
    # training_set = training_set1 + training_set2
    # trainer.train(training_set, 0)
    # trainer.save_features('original_orangeA')
    #
    # detector.set_max_features(750)
    #
    # trainer.load_data('train/pos_info_Courtyard_multi.dat')
    # trainer.evaluate(0)
    # trainer.feature_selection()
    # trainer.evaluate(False)
    # trainer.save_features('orange2_trained')

    # detector = Detector(35)
    #
    # trainer = Trainer()
    # trainer.set_detector(detector)
    # training_set = load_image_folder('train/quadbox/quadbox_black_1')
    # # training_set = load_image_folder('C:/Users/seanr/OneDrive/UAV/data/gazebo')
    # # training_set2 = load_image_folder('train/idea2/orange3')
    # # training_set = training_set1 + training_set2
    # trainer.train_images(training_set, 1)
    # trainer.save_features('original_black')
    #
    # detector.set_max_features(750)
    #
    # trainer.load_data('train/quadbox/pos_info.txt')
    #
    # trainer.evaluate(1)
    # trainer.feature_selection()
    # trainer.evaluate(1)
    # trainer.save_features('black_trained')


if __name__ == "__main__":
    main()