function info = getSymbolsInfo(prach)
%getSymbolsInfo Get PRACH additional information related to the symbol
% generation, as discussed in TS 38.211 Section 6.3.3.1.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%   INFO = getSymbolsInfo(PRACH) returns a structure with these fields:
%
%   RootSequence    - Physical root Zadoff-Chu sequence index or indices (u)
%   CyclicShift     - Cyclic shift or shifts of Zadoff-Chu sequence (C_v)
%   CyclicOffset    - For restricted set mode, cyclic shift or shifts
%                     corresponding to a Doppler shift of 1/T_SEQ (d_u)
%   NumCyclicShifts - Number of cyclic shifts that correspond to a single
%                     PRACH preamble sequence
%
%   PRACH is a PRACH configuration object, <a href="matlab:help('nrPRACHConfig')">nrPRACHConfig</a>.
%   Only these object properties are relevant for this function:
%
%     * FrequencyRange
%     * DuplexMode
%     * ConfigurationIndex
%     * LRA
%     * SequenceIndex
%     * PreambleIndex
%     * RestrictedSet
%     * ZeroCorrelationZone

%  Copyright 2019-2022 The MathWorks, Inc.

%#codegen

    % Set up an empty output vector
    info = struct();
    
    % Get the parameters RootSequence, CyclicShift, CyclicOffset, and NumCyclicShifts
    [rootSequence,cyclicShift,cyclicOffset,numCyclicShifts] = getPreambleSeqParameters(prach);
    info.RootSequence = rootSequence;
    info.CyclicShift = cyclicShift;
    info.CyclicOffset = cyclicOffset;
    info.NumCyclicShifts = numCyclicShifts;

end

function [rootSequence,cyclicShift,cyclicOffset,numCyclicShifts] = getPreambleSeqParameters(prach)
    % Get the values of RootSequence, CyclicShift, CyclicOffset, and
    % NumCyclicShifts.
    %
    % Initialization of variables:
    %  'preIdx': current preamble index, starting at zero and running to
    %            max(PRACH.PreambleIndex), the largest preamble index
    %            requested by the PRACH.PreambleIndex input parameter
    %  'seqIdx': current logical root sequence index, starting at
    %            PRACH.SequenceIndex and incremented when all cyclic shifts
    %            of the physical root sequence index corresponding to the
    %            current 'seqIdx' have been generated
    %   'u_set': set of physical root sequence indices "u"
    % 'd_u_set': set of cyclic shifts "d_u"
    % 'C_v_set': set of cyclic shifts "C_v"
    %     'd_u': cyclic shift "d_u" for the current physical root sequence
    %            index "u" if using the restricted set of cyclic shifts,
    %            otherwise set to 0. For restricted set type A and B, d_u
    %            is initialized to zero and will be updated inside the loop
    %            below for each value of "u".
    preIdx = 0;
    seqIdx = prach.SequenceIndex;
    LRA = prach.LRA;
    d_u = -1 + ~strcmpi(prach.RestrictedSet,'UnrestrictedSet');
    % Preallocate these variables with the maximum size that they can reach
    u_set = zeros(1,110);
    d_u_set = zeros(1,110);
    C_v_set = zeros(1,110);

    numCyclicShifts = nr5g.internal.prach.getNCS(prach.Format,LRA,prach.ZeroCorrelationZone,prach.RestrictedSet);
    
    % Loop over preIdx until preamble sequence parameters for all
    % preamble indices up to max(PRACH.PreambleIndex) have been generated
    while (preIdx <= max(prach.PreambleIndex))
        
        % Wrap seqIdx in the valid range
        seqIdx = mod(seqIdx, LRA-1);
        
        % Get the physical root sequence index 'u' corresponding to the
        % current logical root sequence index 'seqIdx'
        u = getRootSequence(prach,seqIdx);

        % If restricted set is Type A or B, get the cyclic shift 'd_u'
        % corresponding to a Doppler shift of 1/T_SEQ
        if (d_u~=-1)
            d_u = getRestrictedSetCyclicShift(prach,u);
        end

        % Determine 'v' used to generate 'C_v'
        v = getV(prach,numCyclicShifts,d_u);

        % For each value of 'v', generate 'C_v', the value of the 'v'-th
        % cyclic shift of the current physical root sequence 'u', and
        % record them in 'C_v_set' against the preamble indices starting at
        % 'preIdx'
        C_v_set(preIdx+v+1) = getCyclicShifts(prach,numCyclicShifts,d_u,v);

        % Record 'u' in 'u_set' and 'd_u' in 'd_u_set' against the preamble
        % indices starting at 'preIdx'. Note that 'd_u' will be -1
        % if unrestricted set is considered.
        u_set(preIdx+v+1) = u;
        d_u_set(preIdx+v+1) = d_u;

        % Increment logical root sequence index for the next loop iteration
        seqIdx = seqIdx + 1;
        
        % Move to the next sequence index, if this corresponds to an
        % invalid configuration
        if any(C_v_set(preIdx+v+1) == -1)
            continue
        end
        
        % Increment the preamble index
        preIdx = preIdx + v(end) + 1;
    end
    d_u_set(d_u_set==-1) = 0;
    
    rootSequence = u_set(prach.PreambleIndex+1);
    cyclicShift = C_v_set(prach.PreambleIndex+1);
    cyclicOffset = d_u_set(prach.PreambleIndex+1);
    
end

function u = getRootSequence(prach,seqIdx)
    % Get the physical root sequence index 'u' corresponding to the
    % current logical root sequence index 'seqIdx', from TS 38.211
    % Table 6.3.3.1-3 for long preambles and Tables 6.3.3.1-4, 6.3.3.1-4A,
    % and 6.3.3.1-4B for short preambles

    persistent rootsTableLongPreamble;
    persistent rootsTableShortPreamble;
    persistent rootsTableLRA_1151;
    persistent rootsTableLRA_571;

    if (isempty(rootsTableLongPreamble))
        rootsTableShortPreamble = [1 138 2 137 3 136 4 135 5 134 6 133 7 132 8 131 9 130 10 129 11 128 12 127 13 126 14 125 15 124 16 123 17 122 18 121 19 120 20 119 21 118 22 117 23 116 24 115 25 114 26 113 27 112 28 111 29 110 30 109 31 108 32 107 33 106 34 105 35 104 36 103 37 102 38 101 39 100 40 99 41 98 42 97 43 96 44 95 45 94 46 93 47 92 48 91 49 90 50 89 51 88 52 87 53 86 54 85 55 84 56 83 57 82 58 81 59 80 60 79 61 78 62 77 63 76 64 75 65 74 66 73 67 72 68 71 69 70];
        rootsTableLongPreamble = [129 710 140 699 120 719 210 629 168 671 84 755 105 734 93 746 70 769 60 779 2 837 1 838 56 783 112 727 148 691 80 759 42 797 40 799 35 804 73 766 146 693 31 808 28 811 30 809 27 812 29 810 24 815 48 791 68 771 74 765 178 661 136 703 86 753 78 761 43 796 39 800 20 819 21 818 95 744 202 637 190 649 181 658 137 702 125 714 151 688 217 622 128 711 142 697 122 717 203 636 118 721 110 729 89 750 103 736 61 778 55 784 15 824 14 825 12 827 23 816 34 805 37 802 46 793 207 632 179 660 145 694 130 709 223 616 228 611 227 612 132 707 133 706 143 696 135 704 161 678 201 638 173 666 106 733 83 756 91 748 66 773 53 786 10 829 9 830 7 832 8 831 16 823 47 792 64 775 57 782 104 735 101 738 108 731 208 631 184 655 197 642 191 648 121 718 141 698 149 690 216 623 218 621 152 687 144 695 134 705 138 701 199 640 162 677 176 663 119 720 158 681 164 675 174 665 171 668 170 669 87 752 169 670 88 751 107 732 81 758 82 757 100 739 98 741 71 768 59 780 65 774 50 789 49 790 26 813 17 822 13 826 6 833 5 834 33 806 51 788 75 764 99 740 96 743 97 742 166 673 172 667 175 664 187 652 163 676 185 654 200 639 114 725 189 650 115 724 194 645 195 644 192 647 182 657 157 682 156 683 211 628 154 685 123 716 139 700 212 627 153 686 213 626 215 624 150 689 225 614 224 615 221 618 220 619 127 712 147 692 124 715 193 646 205 634 206 633 116 723 160 679 186 653 167 672 79 760 85 754 77 762 92 747 58 781 62 777 69 770 54 785 36 803 32 807 25 814 18 821 11 828 4 835 3 836 19 820 22 817 41 798 38 801 44 795 52 787 45 794 63 776 67 772 72 767 76 763 94 745 102 737 90 749 109 730 165 674 111 728 209 630 204 635 117 722 188 651 159 680 198 641 113 726 183 656 180 659 177 662 196 643 155 684 214 625 126 713 131 708 219 620 222 617 226 613 230 609 232 607 262 577 252 587 418 421 416 423 413 426 411 428 376 463 395 444 283 556 285 554 379 460 390 449 363 476 384 455 388 451 386 453 361 478 387 452 360 479 310 529 354 485 328 511 315 524 337 502 349 490 335 504 324 515 323 516 320 519 334 505 359 480 295 544 385 454 292 547 291 548 381 458 399 440 380 459 397 442 369 470 377 462 410 429 407 432 281 558 414 425 247 592 277 562 271 568 272 567 264 575 259 580 237 602 239 600 244 595 243 596 275 564 278 561 250 589 246 593 417 422 248 591 394 445 393 446 370 469 365 474 300 539 299 540 364 475 362 477 298 541 312 527 313 526 314 525 353 486 352 487 343 496 327 512 350 489 326 513 319 520 332 507 333 506 348 491 347 492 322 517 330 509 338 501 341 498 340 499 342 497 301 538 366 473 401 438 371 468 408 431 375 464 249 590 269 570 238 601 234 605 257 582 273 566 255 584 254 585 245 594 251 588 412 427 372 467 282 557 403 436 396 443 392 447 391 448 382 457 389 450 294 545 297 542 311 528 344 495 345 494 318 521 331 508 325 514 321 518 346 493 339 500 351 488 306 533 289 550 400 439 378 461 374 465 415 424 270 569 241 598 231 608 260 579 268 571 276 563 409 430 398 441 290 549 304 535 308 531 358 481 316 523 293 546 288 551 284 555 368 471 253 586 256 583 263 576 242 597 274 565 402 437 383 456 357 482 329 510 317 522 307 532 286 553 287 552 266 573 261 578 236 603 303 536 356 483 355 484 405 434 404 435 406 433 235 604 267 572 302 537 309 530 265 574 233 606 367 472 296 543 336 503 305 534 373 466 280 559 279 560 419 420 240 599 258 581 229 610];
        rootsTableLRA_1151 = [1,1150,2,1149,3,1148,4,1147,5,1146,6,1145,7,1144,8,1143,9,1142,10,1141,11,1140,12,1139,13,1138,14,1137,15,1136,16,1135,17,1134,18,1133,19,1132,20,1131,21,1130,22,1129,23,1128,24,1127,25,1126,26,1125,27,1124,28,1123,29,1122,30,1121,31,1120,32,1119,33,1118,34,1117,35,1116,36,1115,37,1114,38,1113,39,1112,40,1111,41,1110,42,1109,43,1108,44,1107,45,1106,46,1105,47,1104,48,1103,49,1102,50,1101,51,1100,52,1099,53,1098,54,1097,55,1096,56,1095,57,1094,58,1093,59,1092,60,1091,61,1090,62,1089,63,1088,64,1087,65,1086,66,1085,67,1084,68,1083,69,1082,70,1081,71,1080,72,1079,73,1078,74,1077,75,1076,76,1075,77,1074,78,1073,79,1072,80,1071,81,1070,82,1069,83,1068,84,1067,85,1066,86,1065,87,1064,88,1063,89,1062,90,1061,91,1060,92,1059,93,1058,94,1057,95,1056,96,1055,97,1054,98,1053,99,1052,100,1051,101,1050,102,1049,103,1048,104,1047,105,1046,106,1045,107,1044,108,1043,109,1042,110,1041,111,1040,112,1039,113,1038,114,1037,115,1036,116,1035,117,1034,118,1033,119,1032,120,1031,121,1030,122,1029,123,1028,124,1027,125,1026,126,1025,127,1024,128,1023,129,1022,130,1021,131,1020,132,1019,133,1018,134,1017,135,1016,136,1015,137,1014,138,1013,139,1012,140,1011,141,1010,142,1009,143,1008,144,1007,145,1006,146,1005,147,1004,148,1003,149,1002,150,1001,151,1000,152,999,153,998,154,997,155,996,156,995,157,994,158,993,159,992,160,991,161,990,162,989,163,988,164,987,165,986,166,985,167,984,168,983,169,982,170,981,171,980,172,979,173,978,174,977,175,976,176,975,177,974,178,973,179,972,180,971,181,970,182,969,183,968,184,967,185,966,186,965,187,964,188,963,189,962,190,961,191,960,192,959,193,958,194,957,195,956,196,955,197,954,198,953,199,952,200,951,201,950,202,949,203,948,204,947,205,946,206,945,207,944,208,943,209,942,210,941,211,940,212,939,213,938,214,937,215,936,216,935,217,934,218,933,219,932,220,931,221,930,222,929,223,928,224,927,225,926,226,925,227,924,228,923,229,922,230,921,231,920,232,919,233,918,234,917,235,916,236,915,237,914,238,913,239,912,240,911,241,910,242,909,243,908,244,907,245,906,246,905,247,904,248,903,249,902,250,901,251,900,252,899,253,898,254,897,255,896,256,895,257,894,258,893,259,892,260,891,261,890,262,889,263,888,264,887,265,886,266,885,267,884,268,883,269,882,270,881,271,880,272,879,273,878,274,877,275,876,276,875,277,874,278,873,279,872,280,871,281,870,282,869,283,868,284,867,285,866,286,865,287,864,288,863,289,862,290,861,291,860,292,859,293,858,294,857,295,856,296,855,297,854,298,853,299,852,300,851,301,850,302,849,303,848,304,847,305,846,306,845,307,844,308,843,309,842,310,841,311,840,312,839,313,838,314,837,315,836,316,835,317,834,318,833,319,832,320,831,321,830,322,829,323,828,324,827,325,826,326,825,327,824,328,823,329,822,330,821,331,820,332,819,333,818,334,817,335,816,336,815,337,814,338,813,339,812,340,811,341,810,342,809,343,808,344,807,345,806,346,805,347,804,348,803,349,802,350,801,351,800,352,799,353,798,354,797,355,796,356,795,357,794,358,793,359,792,360,791,361,790,362,789,363,788,364,787,365,786,366,785,367,784,368,783,369,782,370,781,371,780,372,779,373,778,374,777,375,776,376,775,377,774,378,773,379,772,380,771,381,770,382,769,383,768,384,767,385,766,386,765,387,764,388,763,389,762,390,761,391,760,392,759,393,758,394,757,395,756,396,755,397,754,398,753,399,752,400,751,401,750,402,749,403,748,404,747,405,746,406,745,407,744,408,743,409,742,410,741,411,740,412,739,413,738,414,737,415,736,416,735,417,734,418,733,419,732,420,731,421,730,422,729,423,728,424,727,425,726,426,725,427,724,428,723,429,722,430,721,431,720,432,719,433,718,434,717,435,716,436,715,437,714,438,713,439,712,440,711,441,710,442,709,443,708,444,707,445,706,446,705,447,704,448,703,449,702,450,701,451,700,452,699,453,698,454,697,455,696,456,695,457,694,458,693,459,692,460,691,461,690,462,689,463,688,464,687,465,686,466,685,467,684,468,683,469,682,470,681,471,680,472,679,473,678,474,677,475,676,476,675,477,674,478,673,479,672,480,671,481,670,482,669,483,668,484,667,485,666,486,665,487,664,488,663,489,662,490,661,491,660,492,659,493,658,494,657,495,656,496,655,497,654,498,653,499,652,500,651,501,650,502,649,503,648,504,647,505,646,506,645,507,644,508,643,509,642,510,641,511,640,512,639,513,638,514,637,515,636,516,635,517,634,518,633,519,632,520,631,521,630,522,629,523,628,524,627,525,626,526,625,527,624,528,623,529,622,530,621,531,620,532,619,533,618,534,617,535,616,536,615,537,614,538,613,539,612,540,611,541,610,542,609,543,608,544,607,545,606,546,605,547,604,548,603,549,602,550,601,551,600,552,599,553,598,554,597,555,596,556,595,557,594,558,593,559,592,560,591,561,590,562,589,563,588,564,587,565,586,566,585,567,584,568,583,569,582,570,581,571,580,572,579,573,578,574,577,575,576];
        rootsTableLRA_571 = [1,570,2,569,3,568,4,567,5,566,6,565,7,564,8,563,9,562,10,561,11,560,12,559,13,558,14,557,15,556,16,555,17,554,18,553,19,552,20,551,21,550,22,549,23,548,24,547,25,546,26,545,27,544,28,543,29,542,30,541,31,540,32,539,33,538,34,537,35,536,36,535,37,534,38,533,39,532,40,531,41,530,42,529,43,528,44,527,45,526,46,525,47,524,48,523,49,522,50,521,51,520,52,519,53,518,54,517,55,516,56,515,57,514,58,513,59,512,60,511,61,510,62,509,63,508,64,507,65,506,66,505,67,504,68,503,69,502,70,501,71,500,72,499,73,498,74,497,75,496,76,495,77,494,78,493,79,492,80,491,81,490,82,489,83,488,84,487,85,486,86,485,87,484,88,483,89,482,90,481,91,480,92,479,93,478,94,477,95,476,96,475,97,474,98,473,99,472,100,471,101,470,102,469,103,468,104,467,105,466,106,465,107,464,108,463,109,462,110,461,111,460,112,459,113,458,114,457,115,456,116,455,117,454,118,453,119,452,120,451,121,450,122,449,123,448,124,447,125,446,126,445,127,444,128,443,129,442,130,441,131,440,132,439,133,438,134,437,135,436,136,435,137,434,138,433,139,432,140,431,141,430,142,429,143,428,144,427,145,426,146,425,147,424,148,423,149,422,150,421,151,420,152,419,153,418,154,417,155,416,156,415,157,414,158,413,159,412,160,411,161,410,162,409,163,408,164,407,165,406,166,405,167,404,168,403,169,402,170,401,171,400,172,399,173,398,174,397,175,396,176,395,177,394,178,393,179,392,180,391,181,390,182,389,183,388,184,387,185,386,186,385,187,384,188,383,189,382,190,381,191,380,192,379,193,378,194,377,195,376,196,375,197,374,198,373,199,372,200,371,201,370,202,369,203,368,204,367,205,366,206,365,207,364,208,363,209,362,210,361,211,360,212,359,213,358,214,357,215,356,216,355,217,354,218,353,219,352,220,351,221,350,222,349,223,348,224,347,225,346,226,345,227,344,228,343,229,342,230,341,231,340,232,339,233,338,234,337,235,336,236,335,237,334,238,333,239,332,240,331,241,330,242,329,243,328,244,327,245,326,246,325,247,324,248,323,249,322,250,321,251,320,252,319,253,318,254,317,255,316,256,315,257,314,258,313,259,312,260,311,261,310,262,309,263,308,264,307,265,306,266,305,267,304,268,303,269,302,270,301,271,300,272,299,273,298,274,297,275,296,276,295,277,294,278,293,279,292,280,291,281,290,282,289,283,288,284,287,285,286];
    end

    switch prach.LRA
        case 839 % Long preamble
            u = rootsTableLongPreamble(seqIdx+1);
        case 139 % Short preamble - All subcarrier spacing
            u = rootsTableShortPreamble(seqIdx+1);
        case 1151 % Short preamble - 15 kHz
            u = rootsTableLRA_1151(seqIdx+1);
        otherwise % Short preamble - 30 kHz
            u = rootsTableLRA_571(seqIdx+1);
    end
end

function d_u = getRestrictedSetCyclicShift(prach,u)
    % Get the cyclic shift 'd_u' corresponding to a Doppler shift
    % of 1/T_SEQ, as described in TS 38.211 Section 6.3.3.1

    q = 0;
    LRA = prach.LRA;
    while (mod(q*u,LRA)~=1)
        q = q + 1;
    end

    if (q < LRA/2)
        d_u = q;
    else
        d_u = LRA - q;
    end
end

function v = getV(prach,NCS,d_u)
    % Get 'v', needed to generate cyclic shifts 'C_v' of the physical
    % root sequence index 'u', as described in TS 38.211 Section 6.3.3.1
    
    if (d_u==-1) % Unrestricted set
        if (NCS==0)
            v = 0;
        else
            v = 0:floor(prach.LRA/NCS) - 1;
        end
    else % Restricted set type A and B
        [n_shift,d_start,n_group,nbar_shift,nbbar_shift,nbbbar_shift] = getRestrictedCyclicShiftParameters(prach,NCS,d_u);
        if (n_shift==-1 && d_start==-1 && n_group==-1 && nbar_shift==-1)
            v = 0;
        else
            w = n_shift*n_group + nbar_shift;
            if (strcmpi(prach.RestrictedSet,'RestrictedSetTypeA')) % Restricted set type A
                v = 0:w-1;
            else % Restricted set type B
                v = 0:w+nbbar_shift+nbbbar_shift-1;
            end
        end
    end
end

function C_v = getCyclicShifts(prach,NCS,d_u,v)
    % Get 'C_v', the cyclic shift for the 'v'-th shifted sequence with
    % physical root sequence index 'u', as described in TS 38.211 Section
    % 6.3.3.1. 'v' can be a vector

    if (d_u==-1) % Unrestricted set
        C_v = v * NCS;
    else
        if (strcmpi(prach.RestrictedSet,'RestrictedSetTypeA')) % Restricted set type A
            [n_shift,d_start,n_group,nbar_shift] = getRestrictedCyclicShiftParameters(prach,NCS,d_u);
            if (n_shift==-1 && d_start==-1 && n_group==-1 && nbar_shift==-1)
                C_v = -1;
            else
                C_v = d_start*floor(v/n_shift) + mod(v,n_shift)*NCS;
            end
        else % Restricted set type B
            [n_shift,d_start,n_group,nbar_shift,nbbar_shift,~,dbbar_start,dbbbar_start] = getRestrictedCyclicShiftParameters(prach,NCS,d_u);
            if (n_shift==-1 && d_start==-1 && n_group==-1 && nbar_shift==-1)
                C_v = -1;
            else
                w = n_shift*n_group + nbar_shift;
                C_v = zeros(size(v));
                C_v(1:w) = d_start*floor(v(1:w)/n_shift) + mod(v(1:w),n_shift)*NCS;
                C_v(w+1:w+nbbar_shift) = dbbar_start + (v(w+1:w+nbbar_shift) - w)*NCS;
                C_v(w+nbbar_shift+1:end) = dbbbar_start + (v(w+nbbar_shift+1:end) - w - nbbar_shift)*NCS;
            end
        end
    end
end

function [n_shift,d_start,n_group,nbar_shift,nbbar_shift,nbbbar_shift,dbbar_start,dbbbar_start] = ...
    getRestrictedCyclicShiftParameters(prach,NCS,d_u)
    % Get parameters 'n_shift', 'd_start', 'n_group' and 'nbar_shift', used
    % to calculate cyclic shifts 'C_v' of the physical root sequence index
    % 'u' in the case of restricted set, as described in TS 38.211 Section
    % 6.3.3.1. Note that, if there are no cyclic shifts in the restricted
    % set, each parameter will be set to -1.
    % The last four parameters 'nbbar_shift', 'nbbbar_shift',
    % 'dbbar_shift', and 'dbbbar_shift' are needed only for
    % restricted set type B.

    LRA = prach.LRA;
    NCS = NCS(1);

    % Set the default values for the outputs
    n_shift      = -1;
    d_start      = -1;
    n_group      = -1;
    nbar_shift   = -1;
    nbbar_shift  = 0;
    nbbbar_shift = 0;
    dbbar_start  = 0;
    dbbbar_start = 0;

    if isnan(NCS) % Invalid ZeroCorrelationZone
        % Use the default values
    else
        if (strcmpi(prach.RestrictedSet,'RestrictedSetTypeA')) % Restricted set type A
            if (NCS <= d_u && d_u < LRA/3)
                n_shift = floor(d_u/NCS);
                d_start = 2*d_u + n_shift*NCS;
                n_group = floor(LRA/d_start);
                nbar_shift = max(floor((LRA - 2*d_u - n_group*d_start)/NCS),0);
            elseif (LRA/3 <= d_u && d_u <= (LRA - NCS)/2)
                n_shift = floor((LRA - 2*d_u)/NCS);
                d_start = LRA - 2*d_u + n_shift*NCS;
                n_group = floor(d_u/d_start);
                nbar_shift = min(max(floor((d_u - n_group*d_start)/NCS),0),n_shift);
            else
                % Use the default values
            end
        else % Restricted set type B
            if (NCS <= d_u && d_u < LRA/5)
                n_shift = floor(d_u/NCS);
                d_start = 4*d_u + n_shift*NCS;
                n_group = floor(LRA/d_start);
                nbar_shift = max(floor((LRA - 4*d_u - n_group*d_start)/NCS),0);
            elseif (LRA/5 <= d_u && d_u <= (LRA - NCS)/4)
                n_shift = floor((LRA - 4*d_u)/NCS);
                d_start = LRA - 4*d_u + n_shift*NCS;
                n_group = floor(d_u/d_start);
                nbar_shift = min(max(floor((d_u - n_group*d_start)/NCS),0),n_shift);
            elseif ((LRA + NCS)/4 <= d_u && d_u <= 2*LRA/7)
                n_shift = floor((4*d_u - LRA)/NCS);
                d_start = 4*d_u - LRA + n_shift*NCS;
                n_group = floor(d_u/d_start);
                nbar_shift = max(floor((LRA - 3*d_u - n_group*d_start)/NCS),0);
                nbbar_shift = floor(min(d_u - n_group*d_start, 4*d_u - LRA - nbar_shift*NCS)/NCS);
                nbbbar_shift = floor(((1-min(1,nbar_shift))*(d_u - n_group*d_start) + min(1,nbar_shift)*(4*d_u - LRA - nbar_shift*NCS))/NCS) - nbbar_shift;
                dbbar_start = LRA - 3*d_u + n_group*d_start + nbar_shift*NCS;
                dbbbar_start = LRA - 2*d_u + n_group*d_start + nbbar_shift*NCS;
            elseif (2*LRA/7 <= d_u && d_u <= (LRA - NCS)/3)
                n_shift = floor((LRA - 3*d_u)/NCS);
                d_start = LRA - 3*d_u + n_shift*NCS;
                n_group = floor(d_u/d_start);
                nbar_shift = max(floor((4*d_u - LRA - n_group*d_start)/NCS),0);
                nbbar_shift = floor(min(d_u - n_group*d_start,LRA - 3*d_u - nbar_shift*NCS)/NCS);
                nbbbar_shift = 0;
                dbbar_start = d_u + n_group*d_start + nbar_shift*NCS;
                dbbbar_start = 0;
            elseif ((LRA + NCS)/3 <= d_u && d_u < 2*LRA/5)
                n_shift = floor((3*d_u - LRA)/NCS);
                d_start = 3*d_u - LRA + n_shift*NCS;
                n_group = floor(d_u/d_start);
                nbar_shift = max(floor((LRA - 2*d_u - n_group*d_start)/NCS),0);
            elseif (2*LRA/5 <= d_u && d_u <= (LRA - NCS)/2)
                n_shift = floor((LRA - 2*d_u)/NCS);
                d_start = 2*(LRA - 2*d_u) + n_shift*NCS;
                n_group = floor((LRA - d_u)/d_start);
                nbar_shift = max(floor((3*d_u - LRA - n_group*d_start)/NCS),0);
            else
                % Use the default values
            end
        end
    end
end
