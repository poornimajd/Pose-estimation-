import pickle                                                  #Enter proper paths here for the poseestimation
cnet_path = "/poseestimation/src/lib/"

model_path = "/poseestimation/models/multi_pose_dla_3x.pth"
source_path = "/poseestimation/src/floorfirst.avi"

downrail = [[391, 1], [389, 2], [389, 3], [388, 3], [387, 3], [386, 4], [385, 5], [384, 6], [384, 6], [383, 7], [382, 7], [382, 8], [381, 9], [381, 9], [381, 10], [380, 10], [380, 11], [380, 11], [379, 12], [378, 13], [377, 14], [377, 14], [377, 15], [376, 16], [375, 16], [375, 17], [374, 17], [374, 17], [373, 18], [372, 19], [371, 20], [370, 21], [369, 21], [369, 21], [368, 22], [367, 22], [366, 22], [365, 23], [365, 24], [364, 24], [363, 25], [363, 26], [362, 26], [361, 26], [361, 26], [360, 27], [359, 27], [359, 27], [358, 28], [357, 28], [357, 29], [355, 30], [354, 31], [354, 31], [353, 32], [351, 34], [350, 35], [349, 36], [348, 36], [347, 37], [345, 38], [344, 39], [344, 39], [342, 40], [341, 41], [340, 42], [339, 42], [338, 43], [338, 43], [337, 43], [336, 44], [335, 44], [334, 45], [334, 46], [331, 47], [329, 48], [328, 49], [326, 50], [325, 51], [324, 51], [323, 51], [321, 52], [320, 53], [319, 54], [319, 55], [318, 56], [318, 57], [317, 58], [316, 59], [315, 61], [315, 61], [315, 62], [314, 62], [313, 64], [312, 64], [312, 65], [311, 66], [310, 67], [309, 68], [308, 68], [307, 69], [307, 69], [307, 70], [307, 71], [306, 72], [306, 73], [305, 75], [304, 76], [303, 77], [302, 78], [301, 79], [300, 80], [299, 81], [298, 81], [298, 82], [297, 82], [296, 83], [295, 84], [294, 85], [294, 85], [293, 86], [292, 87], [291, 88], [290, 89], [289, 91], [288, 91], [287, 92], [286, 93], [284, 95], [283, 96], [283, 96], [282, 97], [281, 98], [280, 99], [279, 100], [278, 101], [278, 103], [277, 104], [275, 106], [274, 106], [274, 108], [274, 109], [273, 110], [273, 111], [272, 111], [272, 112], [271, 113], [270, 114], [270, 115], [269, 116], [268, 116], [267, 117], [266, 118], [266, 118], [266, 119], [266, 120], [265, 121], [264, 121], [263, 122], [263, 123], [262, 123], [261, 124], [260, 125], [260, 125], [260, 126], [260, 127], [260, 127], [260, 129], [259, 130], [259, 131], [258, 132], [258, 133], [257, 133], [257, 134], [256, 135], [256, 136], [256, 137], [255, 138], [255, 139], [254, 140], [253, 141], [252, 142], [251, 143], [251, 143], [251, 144], [250, 145], [249, 145], [248, 146], [248, 148], [246, 148], [245, 148], [244, 149], [241, 150], [240, 151], [240, 152], [239, 153], [237, 154], [236, 154], [235, 156], [235, 157], [234, 158], [234, 159], [233, 159], [232, 160], [231, 161], [230, 162], [229, 163], [228, 164], [227, 165], [227, 166], [226, 167], [226, 168], [225, 169], [224, 170], [223, 171], [222, 172], [222, 173], [221, 174], [219, 175], [218, 176], [218, 178], [217, 179], [216, 181], [216, 183], [215, 184], [213, 186], [213, 188], [212, 189], [211, 190], [211, 191], [210, 192], [209, 193], [208, 194], [208, 195], [207, 196], [207, 198], [206, 199], [205, 201], [204, 202], [203, 205], [202, 207], [201, 207], [199, 209], [199, 211], [198, 213], [198, 214], [197, 216], [197, 218], [197, 219], [196, 220], [195, 221], [194, 222], [194, 223], [193, 224], [193, 225], [192, 226], [192, 227], [192, 228], [191, 229], [190, 230], [189, 231], [188, 232], [188, 233], [187, 234], [185, 235], [185, 236], [184, 237], [184, 238], [183, 240], [183, 241], [182, 242], [181, 243], [181, 244], [180, 245], [179, 246], [178, 248], [176, 249], [176, 250], [176, 252], [175, 253], [174, 254], [172, 256], [171, 257], [171, 257], [171, 259], [171, 260], [170, 262], [170, 262], [169, 263], [168, 264], [167, 265], [167, 266], [166, 267], [165, 269], [164, 273], [163, 274], [162, 275], [162, 276], [161, 277], [160, 278], [159, 279], [158, 280], [157, 281], [157, 282], [156, 283], [155, 285], [155, 287], [154, 288], [152, 290], [151, 291], [151, 292], [150, 294], [150, 296], [149, 298], [148, 299], [147, 300], [147, 301], [146, 303], [146, 304], [146, 305], [146, 306], [145, 308], [145, 310], [145, 311], [145, 312], [145, 313], [145, 315], [145, 316], [145, 316], [145, 318], [144, 319], [144, 320], [143, 321], [143, 321], [143, 323], [142, 324], [142, 325], [141, 326], [141, 327], [141, 328], [140, 329], [140, 330], [140, 331], [139, 332], [139, 332], [139, 334], [139, 334], [139, 335], [139, 336], [139, 337], [139, 338], [139, 338], [140, 340], [140, 341], [140, 342], [140, 342], [140, 344], [140, 344], [140, 345], [140, 346], [140, 347], [140, 348], [140, 348], [140, 349], [140, 350], [140, 351], [140, 352], [140, 354], [139, 354], [139, 355], [139, 356], [139, 357], [139, 358], [140, 359], [140, 360], [140, 362], [141, 362], [141, 363], [141, 364], [141, 365], [141, 365], [141, 366], [141, 367], [141, 368], [141, 368], [141, 370], [141, 370], [141, 371], [141, 371], [141, 373], [141, 373], [142, 374], [142, 375], [142, 377], [143, 378], [143, 380], [144, 380], [144, 381], [144, 382], [145, 383], [145, 384], [145, 385], [145, 386], [146, 387], [146, 389], [146, 390], [146, 391], [146, 392], [148, 393], [148, 394], [149, 395], [149, 396], [149, 397], [150, 399], [151, 400], [152, 401], [152, 402], [152, 403], [152, 404], [152, 405], [152, 407], [153, 409], [153, 409], [154, 412], [155, 414], [155, 415], [155, 416], [155, 417], [155, 418], [155, 419], [155, 420], [155, 421], [155, 421], [155, 423], [155, 424], [156, 424], [156, 425], [157, 426], [157, 427], [157, 427], [157, 428], [157, 429], [158, 430], [158, 431], [158, 432], [158, 432], [159, 434], [159, 435], [159, 435], [159, 436], [159, 437], [160, 438], [161, 438], [161, 439], [161, 440], [161, 441], [161, 442], [162, 443], [162, 443], [162, 445], [162, 445], [163, 447], [164, 447], [164, 448], [164, 450], [165, 451], [165, 452], [165, 453], [165, 454], [166, 455], [166, 456], [166, 457], [166, 457], [166, 458], [167, 461], [167, 462], [167, 462], [168, 463], [168, 464], [168, 465], [168, 465], [168, 466], [168, 468], [169, 469], [169, 469], [169, 470], [170, 473], [170, 474], [170, 475], [170, 476], [170, 477], [170, 478], [171, 480], [172, 481], [172, 482], [172, 483], [173, 485], [173, 486], [174, 487], [174, 488], [175, 490], [175, 490], [175, 492], [176, 493], [177, 494], [177, 495], [177, 495], [178, 496], [178, 497], [178, 499], [179, 500], [179, 501], [181, 503], [181, 504], [181, 505], [181, 506], [181, 507], [182, 508], [182, 509], [183, 510], [183, 511], [183, 512], [183, 512], [183, 513], [184, 514], [184, 515], [185, 516], [185, 517], [186, 518], [186, 519], [186, 520], [186, 520], [186, 523], [186, 523], [186, 524], [187, 526], [187, 526], [187, 527], [188, 529], [188, 529], [189, 531], [189, 532], [189, 534], [189, 535], [189, 536], [189, 537], [190, 538], [190, 539], [191, 540], [191, 542], [191, 542], [191, 543], [193, 545], [193, 546], [194, 547], [194, 548], [195, 549], [195, 550], [196, 552], [197, 553], [197, 554], [197, 555], [198, 556], [199, 558], [199, 558], [200, 559], [200, 560], [201, 561], [202, 563], [203, 563], [203, 564], [203, 566], [203, 566], [204, 567], [204, 568], [204, 569], [205, 570], [205, 571], [205, 572], [204, 569], [201, 566], [633, 2], [633, 2], [632, 2], [632, 2], [631, 2], [631, 2], [630, 3], [629, 4], [628, 4], [627, 5], [626, 6], [625, 8], [625, 9], [625, 10], [625, 11], [624, 11], [624, 12], [624, 13], [624, 14], [624, 14], [624, 15], [624, 16], [624, 17], [624, 17], [624, 18], [624, 19], [623, 19], [622, 20], [622, 20], [622, 22], [622, 23], [621, 23], [621, 24], [621, 24], [621, 26], [621, 27], [621, 27], [619, 29], [619, 30], [619, 31], [619, 32], [618, 33], [618, 34], [617, 35], [617, 36], [617, 36], [617, 38], [616, 39], [616, 40], [615, 41], [615, 41], [615, 42], [614, 43], [614, 44], [613, 45], [613, 46], [612, 47], [612, 48], [612, 48], [611, 49], [611, 49], [610, 51], [610, 51], [610, 52], [609, 53], [609, 54], [608, 55], [607, 56], [607, 58], [607, 58], [606, 60], [605, 61], [604, 62], [603, 64], [602, 66], [602, 67], [601, 68], [600, 71], [599, 72], [599, 73], [599, 74], [599, 75], [598, 77], [598, 79], [598, 80], [598, 80], [596, 82], [596, 84], [596, 85], [595, 87], [595, 88], [595, 89], [595, 89], [595, 90], [595, 92], [595, 93], [594, 94], [593, 95], [592, 95], [592, 96], [591, 98], [590, 99], [590, 101], [590, 102], [590, 102], [590, 104], [590, 104], [590, 105], [589, 106], [588, 107], [588, 108], [588, 108], [588, 109], [588, 110], [588, 110], [588, 111], [587, 111], [587, 112], [587, 113], [587, 114], [586, 115], [585, 115], [585, 116], [585, 117], [585, 117], [585, 118], [585, 120], [585, 121], [585, 121], [584, 122], [584, 123], [584, 124], [584, 124], [584, 125], [583, 128], [583, 128], [583, 129], [583, 131], [583, 132], [582, 133], [581, 134], [581, 135], [581, 136], [580, 137], [580, 138], [580, 141], [580, 142], [579, 143], [579, 145], [579, 146], [579, 146], [579, 147], [579, 149], [579, 150], [579, 150], [579, 151], [579, 153], [579, 153], [579, 154], [579, 155], [579, 156], [579, 157], [579, 158], [579, 160], [579, 160], [579, 161], [579, 162], [579, 163], [579, 164], [579, 165], [579, 165], [578, 167], [577, 168], [576, 169], [576, 170], [576, 171], [576, 172], [575, 173], [575, 175], [575, 176], [574, 176], [573, 177], [572, 178], [572, 179], [572, 180], [572, 181], [572, 182], [572, 183], [572, 184], [572, 185], [572, 186], [572, 187], [572, 188], [572, 188], [572, 189], [572, 191], [572, 191], [572, 192], [572, 193], [572, 194], [572, 195], [572, 196], [572, 197], [572, 198], [572, 199], [572, 200], [572, 200], [572, 202], [572, 203], [571, 204], [571, 204], [571, 206], [571, 207], [571, 207], [571, 208], [571, 210], [571, 210], [571, 211], [571, 212], [571, 214], [571, 214], [571, 215], [571, 216], [571, 217], [571, 218], [571, 219], [571, 219], [569, 221], [569, 223], [569, 225], [569, 227], [569, 228], [569, 230], [569, 231], [569, 232], [569, 233], [569, 235], [569, 236], [570, 238], [570, 240], [570, 241], [570, 242], [570, 243], [570, 245], [570, 246], [570, 247], [570, 248], [570, 249], [570, 250], [570, 251], [570, 252], [570, 253], [570, 254], [570, 255], [570, 256], [570, 257], [570, 258], [570, 258], [570, 260], [570, 261], [570, 262], [570, 263], [570, 264], [570, 265], [570, 266], [570, 267], [570, 268], [570, 269], [570, 270], [570, 271], [570, 272], [570, 273], [571, 274], [571, 275], [571, 276], [571, 277], [572, 279], [573, 279], [573, 280], [573, 281], [574, 282], [575, 284], [575, 285], [575, 286], [575, 287], [575, 288], [575, 289], [575, 289], [576, 290], [576, 291], [576, 293], [576, 294], [576, 295], [576, 296], [576, 298], [577, 300], [578, 301], [578, 302], [578, 303], [578, 303], [578, 304], [578, 306], [578, 307], [578, 308], [578, 309], [578, 310], [578, 311], [578, 311], [578, 312], [578, 313], [578, 315], [578, 315], [579, 316], [579, 317], [580, 319], [581, 320], [581, 322], [581, 322], [582, 323], [582, 323], [582, 325], [582, 325], [582, 326], [582, 327], [582, 328], [582, 329], [582, 329], [582, 330], [582, 331], [582, 332], [582, 332], [583, 333], [583, 334], [584, 336], [585, 337], [585, 338], [585, 339], [585, 339], [585, 340], [585, 341], [585, 341], [585, 342], [586, 343], [587, 343], [587, 345], [588, 346], [589, 348], [589, 348], [589, 349], [592, 351], [593, 352], [593, 352], [594, 353], [594, 354], [595, 355], [595, 357], [595, 357], [596, 358], [596, 359], [595, 360], [595, 361], [595, 361], [595, 362], [595, 363], [595, 364], [595, 364], [595, 365], [595, 366], [595, 367], [595, 368], [595, 369], [595, 370], [595, 372], [595, 374], [595, 376], [595, 377], [595, 377], [595, 378], [595, 380], [596, 381], [596, 382], [597, 385], [597, 387], [598, 387], [598, 389], [598, 390], [598, 391], [599, 392], [599, 394], [600, 395], [601, 395], [602, 397], [603, 399], [603, 400], [604, 401], [606, 403], [606, 403], [606, 404], [606, 406], [606, 407], [606, 407], [606, 408], [607, 410], [609, 412], [610, 413], [610, 413], [610, 414], [610, 416], [610, 416], [610, 417], [610, 419], [610, 419], [611, 421], [611, 422], [611, 423], [611, 423], [613, 425], [613, 426], [613, 427], [613, 428], [614, 430], [614, 431], [615, 432], [615, 433], [616, 434], [616, 434], [616, 435], [616, 436], [616, 437], [617, 438], [618, 439], [619, 439], [619, 440], [619, 441], [619, 442], [620, 443], [621, 443], [622, 444], [622, 445], [622, 446], [622, 447], [622, 448], [622, 448], [622, 449], [623, 450], [623, 450], [623, 451], [623, 452], [624, 453], [624, 454], [624, 455], [624, 456], [625, 457], [625, 457], [626, 458], [626, 459], [626, 459], [628, 461], [629, 462], [629, 463], [629, 464], [629, 464], [630, 465], [632, 467], [633, 468], [633, 469], [633, 469], [634, 471], [635, 472], [636, 473], [637, 474], [637, 475], [638, 476], [638, 477], [638, 477], [639, 480], [640, 481], [641, 481], [641, 482], [643, 483], [643, 484], [643, 485], [643, 486]]


uprail = [[21, 233], [21, 233], [23, 233], [24, 233], [25, 233], [26, 233], [27, 233], [28, 233], [30, 233], [34, 235], [35, 235], [36, 235], [37, 235], [38, 235], [40, 235], [42, 235], [44, 235], [45, 235], [46, 235], [47, 235], [49, 235], [51, 234], [52, 234], [56, 232], [60, 230], [64, 229], [66, 228], [69, 228], [70, 227], [72, 227], [75, 225], [77, 225], [80, 225], [81, 225], [84, 225], [87, 224], [90, 223], [92, 222], [95, 222], [96, 222], [99, 222], [100, 222], [101, 222], [102, 221], [104, 220], [105, 220], [107, 220], [109, 220], [110, 219], [111, 219], [113, 218], [114, 218], [115, 218], [116, 218], [118, 217], [119, 217], [122, 217], [124, 215], [127, 214], [129, 214], [130, 214], [132, 213], [135, 213], [136, 213], [137, 213], [140, 212], [143, 210], [144, 210], [146, 210], [147, 209], [149, 209], [150, 209], [152, 208], [153, 208], [154, 207], [155, 207], [156, 207], [158, 207], [159, 207], [160, 207], [162, 206], [165, 205], [167, 204], [168, 204], [170, 204], [171, 204], [173, 204], [174, 203], [175, 203], [178, 203], [179, 203], [180, 202], [181, 202], [183, 202], [184, 202], [185, 202], [186, 202], [189, 201], [190, 200], [193, 199], [194, 199], [195, 199], [198, 198], [199, 198], [201, 198], [205, 198], [206, 197], [208, 197], [210, 196], [211, 195], [212, 195], [213, 195], [215, 194], [216, 194], [219, 194], [220, 194], [221, 194], [224, 193], [225, 193], [226, 193], [230, 193], [231, 193], [232, 192], [233, 192], [234, 192], [235, 192], [238, 192], [239, 192], [242, 190], [243, 190], [245, 190], [246, 189], [248, 189], [249, 189], [250, 189], [251, 189], [253, 189], [255, 188], [256, 188], [258, 188], [259, 188], [260, 188], [261, 188], [264, 187], [265, 187], [268, 187], [272, 186], [273, 185], [275, 184], [279, 184], [280, 184], [281, 184], [283, 184], [284, 184], [285, 184], [288, 183], [291, 183], [295, 183], [298, 183], [299, 183], [304, 183], [306, 183], [310, 183], [311, 183], [313, 183], [315, 183], [316, 183], [318, 183], [319, 183], [321, 183], [324, 183], [325, 183], [326, 183], [329, 183], [330, 183], [333, 183], [335, 183], [336, 183], [339, 183], [340, 183], [341, 183], [346, 183], [348, 183], [350, 183], [351, 183], [355, 183], [358, 183], [360, 183], [361, 183], [363, 183], [365, 183], [368, 182], [369, 182], [371, 182], [374, 182], [377, 180], [379, 180], [381, 179], [384, 179], [386, 178], [388, 178], [389, 177], [390, 177], [391, 177], [393, 175], [394, 175], [396, 174], [400, 173], [404, 172], [406, 172], [409, 172], [411, 170], [414, 170], [416, 170], [420, 169], [423, 169], [424, 168], [425, 168], [426, 168], [428, 167], [430, 167], [434, 166], [436, 164], [440, 162], [442, 160], [445, 159], [449, 159], [451, 158], [454, 157], [455, 157], [459, 156], [460, 155], [462, 154], [464, 154], [466, 152], [468, 152], [470, 151], [472, 149], [474, 148], [476, 148], [477, 147], [478, 147], [479, 147], [480, 146], [481, 145], [482, 144], [484, 144], [485, 143], [487, 141], [487, 140], [488, 139], [489, 139], [490, 138], [491, 138], [493, 137], [495, 136], [496, 135], [497, 134], [499, 133], [500, 132], [503, 130], [505, 129], [506, 128], [508, 127], [510, 126], [511, 125], [512, 124], [513, 123], [514, 122], [516, 120], [517, 119], [518, 118], [519, 117], [521, 115], [521, 114], [522, 113], [523, 112], [524, 111], [525, 110], [526, 109], [527, 106], [528, 105], [529, 104], [530, 103], [531, 102], [531, 102], [531, 101], [532, 100], [533, 98], [533, 97], [533, 97], [533, 95], [533, 95], [533, 94], [533, 93], [534, 91], [534, 90], [534, 88], [534, 88], [535, 87], [535, 86], [535, 86], [535, 84], [536, 84], [537, 82], [537, 81], [537, 81], [537, 80], [537, 79], [537, 78], [537, 77], [537, 77], [537, 75], [537, 75], [538, 74], [539, 73], [540, 71], [540, 70], [540, 69], [540, 68], [540, 67], [540, 66], [540, 65], [540, 64], [540, 63], [540, 62], [540, 62], [540, 60], [540, 59], [540, 59], [539, 58], [539, 58], [537, 56], [537, 56], [536, 56], [536, 55], [535, 55], [534, 55], [533, 53], [532, 52], [531, 52], [530, 51], [529, 50], [528, 49], [527, 49], [526, 47], [525, 46], [524, 45], [523, 45], [522, 44], [522, 44], [521, 42], [520, 42], [519, 42], [519, 41], [518, 40], [518, 40], [517, 39], [743, 1], [744, 1], [745, 1], [746, 2], [747, 3], [747, 3], [747, 4], [747, 4], [747, 5], [748, 5], [749, 6], [749, 7], [750, 8], [750, 8], [751, 9], [751, 9], [752, 10], [753, 10], [756, 10], [757, 11], [757, 13], [758, 13], [759, 14], [760, 15], [760, 16], [761, 17], [762, 17], [762, 18], [762, 19], [763, 19], [763, 20], [764, 20], [765, 21], [766, 22], [767, 23], [768, 23], [769, 23], [770, 24], [770, 24], [770, 25], [771, 26], [771, 27], [772, 28], [773, 29], [774, 30], [775, 31], [775, 32], [776, 32], [777, 33], [778, 34], [779, 35], [780, 36], [781, 37], [782, 38], [783, 39], [783, 40], [784, 41], [785, 42], [786, 42], [787, 44], [788, 45], [788, 45], [789, 46], [789, 47], [790, 48], [790, 48], [791, 50], [792, 51], [793, 52], [794, 52], [795, 53], [797, 55], [798, 56], [799, 57], [800, 59], [801, 60], [802, 61], [803, 62], [804, 64], [805, 65], [805, 66], [805, 67], [807, 68], [807, 69], [807, 70], [807, 70], [807, 72], [808, 73], [808, 74], [809, 74], [810, 75], [810, 76], [811, 77], [812, 78], [813, 78], [813, 79], [813, 80], [814, 81], [815, 82], [816, 83], [817, 83], [818, 84], [818, 85], [820, 86], [821, 87], [822, 87], [823, 88], [823, 88], [824, 90], [825, 91], [825, 91], [825, 92], [825, 93], [826, 94], [826, 95], [826, 95], [827, 96], [827, 96], [827, 98], [828, 99], [828, 99], [828, 100], [829, 101], [830, 101], [830, 102], [830, 103], [831, 104], [832, 104], [832, 105], [833, 106], [834, 107], [835, 108], [835, 109], [836, 109], [836, 111], [837, 112], [837, 112], [838, 113], [839, 114], [839, 116], [839, 117], [840, 117], [841, 118], [842, 118], [842, 120], [842, 121], [842, 121], [843, 122], [843, 122], [843, 123], [844, 123], [844, 124], [844, 125], [844, 126], [844, 126], [844, 128], [845, 129], [848, 129], [848, 131], [849, 132], [849, 133], [849, 134], [849, 135], [850, 137], [850, 138], [850, 139], [850, 140], [850, 140], [850, 141], [850, 142], [851, 143], [851, 144], [852, 145], [853, 146], [853, 147], [853, 147], [853, 148], [853, 149], [854, 150], [855, 151], [855, 153], [855, 153], [856, 154], [856, 155], [857, 157], [858, 158], [858, 159], [859, 160], [859, 161], [860, 161], [860, 163], [860, 163], [860, 164], [861, 165], [862, 166], [863, 167], [863, 168], [864, 170], [864, 171], [866, 172], [866, 173], [866, 174], [867, 175], [867, 176], [868, 177], [868, 178], [868, 179], [868, 180], [868, 181], [870, 183], [870, 183], [870, 184], [870, 185], [870, 187], [870, 188], [871, 189], [872, 192], [872, 193], [872, 194], [872, 194], [872, 195], [872, 196], [872, 197], [872, 198], [873, 200], [874, 201], [874, 202], [874, 204], [875, 206], [875, 206], [875, 208], [875, 209], [876, 210], [877, 211], [877, 213], [877, 214], [878, 216], [879, 217], [879, 218], [880, 220], [880, 223], [880, 225], [881, 226], [881, 228], [881, 230], [882, 231], [882, 233], [882, 234], [882, 235], [882, 238], [882, 239], [882, 240], [882, 241], [882, 243], [882, 244], [882, 245], [882, 246], [882, 248], [882, 248], [882, 249], [882, 250], [882, 252], [882, 253], [882, 253], [882, 254], [882, 256], [882, 256], [882, 258], [882, 259], [882, 260], [882, 261], [882, 262], [882, 263], [882, 264], [882, 265], [882, 267], [882, 268], [882, 269], [882, 270], [882, 272], [882, 273], [882, 274], [882, 275], [882, 277], [882, 278], [882, 279], [882, 280], [882, 282], [882, 283], [882, 284], [882, 285], [882, 286], [881, 287], [881, 288], [881, 289], [881, 290], [881, 291], [879, 294], [879, 295], [879, 296], [879, 297], [878, 298], [878, 300], [878, 301], [878, 302], [878, 303], [878, 304], [877, 305], [877, 306], [877, 307], [877, 309], [877, 310], [877, 311], [877, 313], [877, 314], [877, 315], [877, 316], [877, 318], [877, 319], [877, 320], [877, 322], [877, 323], [877, 324], [876, 325], [876, 327], [876, 328], [875, 329], [874, 330], [874, 332], [874, 333], [874, 333], [874, 334], [874, 336], [874, 338], [874, 338], [874, 340], [874, 341], [874, 342], [874, 342], [874, 345], [874, 346], [874, 347], [873, 348], [873, 349], [873, 350], [873, 351], [873, 353], [873, 354], [872, 355], [871, 357], [870, 358], [870, 360], [869, 361], [869, 363], [868, 365], [867, 367], [867, 369], [867, 370], [866, 372], [865, 373], [865, 374], [865, 375], [864, 377], [864, 378], [864, 379], [863, 382], [863, 384], [863, 385], [862, 387], [860, 390], [859, 392], [859, 394], [858, 396], [857, 397], [857, 398], [856, 400], [854, 402], [853, 404], [852, 406], [852, 408], [850, 411], [849, 412], [848, 414], [847, 416], [845, 418], [844, 420], [843, 423], [842, 425], [841, 427], [840, 428], [839, 430], [838, 431], [837, 433], [835, 436], [834, 438], [833, 439], [833, 440], [832, 441], [830, 445], [829, 446], [828, 448], [827, 449], [825, 451], [824, 453], [824, 454], [823, 455], [822, 458], [820, 460], [819, 461], [818, 462], [817, 464], [816, 465], [815, 466], [814, 467], [813, 469], [812, 471], [812, 472], [810, 474], [809, 475], [808, 476], [807, 477], [806, 479], [805, 480], [804, 481], [804, 482], [803, 483], [802, 485], [800, 486], [800, 487], [800, 488], [799, 489], [798, 491], [796, 493], [795, 494], [794, 494], [794, 495], [794, 496], [793, 498], [792, 499], [791, 499], [790, 500], [789, 501], [789, 502], [788, 503], [787, 503], [785, 504], [785, 505], [783, 506], [782, 508], [782, 509], [782, 510], [781, 511], [780, 512], [779, 513], [779, 514], [778, 515], [777, 515], [776, 518], [775, 519], [774, 520], [773, 521], [773, 522], [772, 523], [771, 524], [770, 526], [768, 528], [767, 529], [766, 530], [765, 532], [764, 533], [763, 534], [762, 535], [760, 536], [760, 537], [760, 538], [759, 538], [758, 539], [758, 540], [757, 541], [756, 542], [756, 542], [755, 543], [755, 544], [754, 545], [752, 547], [750, 547], [750, 548], [748, 550], [746, 551], [743, 551], [742, 552], [742, 553], [741, 554], [740, 555], [740, 556], [739, 556], [738, 557], [737, 558], [737, 559], [736, 560], [735, 563], [733, 563], [732, 564], [731, 564], [730, 564], [729, 565], [729, 566], [727, 567], [727, 568]]

