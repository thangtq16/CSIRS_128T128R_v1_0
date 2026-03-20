classdef packetTags
    %PacketTags This class serves as a utility for managing the tags associated
    %with packets. This class includes a suite of static methods aimed at
    %creating tags and operating on tags.
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases
    %
    %   packetTags implements the functionality required to create tags and
    %   operating on tags.
    %
    %   packetTags static methods:
    %
    %   add       - Append an additional tag to a list of existing tags
    %   remove    - Remove a tag matching with given name
    %   find      - Return a tag matching with given name
    %   append    - Append multiple list of tags into a single list
    %   adjust    - Update byte range of tags by a specified offset
    %   segment   - Adjust tags for a specific packet segment
    %   aggregate - Combine tags for packet aggregation
    %   merge     - Merge tags with same name for packet reassembly

    %   Copyright 2024 The MathWorks, Inc.

    methods(Static)
        function updatedTags = add(tags, name, value, byteRange)
            %add Append an additional tag to a list of existing tags
            %
            %   UPDATEDTAGS = add(TAGS, NAME, BYTERANGE, VALUE) appends an
            %   additional tag to TAGS.
            %
            %   UPDATEDTAGS is a list of tags where each tag is a structure with
            %   the fields.
            %       Name      - Name of the tag.
            %       Value     - Data associated with the tag.
            %       ByteRange - Range of bytes within the packet to which the
            %                   tag is applicable.
            %
            %   TAGS is a list of existing tags corresponds to a packet.
            %
            %   NAME is the name of the new tag being added.
            %
            %   VALUE is the data associated with the new tag.
            %
            %   BYTERANGE is the specific range of bytes within the packet that
            %   the new tag applies. This is a 2-element vector.

            % Validate the input arguments
            arguments
                tags (1, :)
                name {mustBeTextScalar}
                value
                byteRange (1,2)
            end
            validateattributes(byteRange,{'numeric'},{'nondecreasing'},'packetTags.add','byteRange')

            % Define a persistent variable to hold the format of a tag
            persistent tagFormat;
            % Initialize the tag format if it has not been initialized yet
            if isempty(tagFormat)
                tagFormat = struct("Name", [], "Value", [], "ByteRange", []);
            end

            % Create a new tag based on the provided name, value, and byte range
            newTag = tagFormat;
            newTag.Name = name;
            newTag.ByteRange = byteRange;
            newTag.Value = value;
            % Append the new tag to the existing list of tags
            updatedTags = [tags newTag];
        end

        function [tags, removedTag] = remove(tags, name)
            %remove Remove the tag matching with given name
            %
            %   [TAGS, REMOVEDTAG] = remove(TAGS, NAME) removes the tag with the
            %   specified name, NAME, from the given tags, TAGS.
            %
            %   TAGS is a list of tags where each tag is a structure with the
            %   fields.
            %       Name      - Name of the tag.
            %       Value     - Data associated with the tag.
            %       ByteRange - Range of bytes within the packet to which the
            %                   tag is applicable.
            %
            %   REMOVEDTAG is a tag which is removed from the given tags, TAGS.
            %
            %   NAME is the name of the tag to be removed from the given tags,
            %   TAGS.

            % Validate the input arguments
            arguments
                tags (1,:)
                name {mustBeTextScalar}
            end

            removedTag = [];
            % Find and remove the first tag with the matching name
            for idx = 1:numel(tags)
                if strcmp(tags(idx).Name, name)
                    % Extract the tag to be removed
                    removedTag = tags(idx);
                    % Remove the matching tag from the original list of tags
                    tags(idx) = [];
                    % Exit the loop as soon as a match is removed
                    break;
                end
            end
        end

        function matchingTag = find(tags, name)
            %find Return tag matching with given name
            %
            %   MATCHINGTAG = find(TAGS, NAME) finds and returns a tag within
            %   the TAGS list that has a name field matching NAME.
            %
            %   TAGS is a tag where each tag is a structure with the fields.
            %       Name      - Name of the tag.
            %       Value     - Data associated with the tag.
            %       ByteRange - Range of bytes within the packet to which the
            %                   tag is applicable.
            %
            %   NAME is the name of the tag to be located within TAGS.
            %
            %   MATCHINGTAG contains a tag from TAGS that have a name field
            %   matching NAME.

            % Validate the input arguments
            arguments
                tags (1,:)
                name {mustBeTextScalar}
            end

            matchingTag = [];
            % Find the first tag with the matching name
            for idx = 1:numel(tags)
                if strcmp(tags(idx).Name, name) % Compare tag name with the given name
                    % Extract the matching tag
                    matchingTag = tags(idx);
                    % Exit the loop as soon as a match is found
                    break;
                end
            end
        end

        function tagList = append(varargin)
            %append Append multiple list of tags into a single list
            %
            %   TAGLIST = append(TAGS1, TAGS2, ..., TAGSN) combines the given
            %   list of tags into a single list.
            %
            %   TAGLIST is a combined list of tags, containing all tags from the
            %   input lists in the order they were provided.
            %
            %   TAGS1, TAGS, ..., TAGSN represent a list of tags where each tag
            %   is a structure with the fields.
            %       Name      - Name of the tag.
            %       Value     - Data associated with the tag.
            %       ByteRange - Range of bytes within the packet to which the
            %                   tag is applicable.

            % Validate the input arguments
            arguments(Repeating)
                varargin (1, :)
            end

            % Initialize an empty array to hold the combined list of tags.
            tagList = [];

            % Iterate over each input argument (each list of tags) and
            % concatenate them into tagList
            for idx = 1:nargin
                % Combine the two list of tags into one list
                tagList = [tagList varargin{idx}];
            end
        end

        function tags = adjust(tags, offset)
            %adjust Update byte range of tags by a specified offset
            %
            %   TAGS = adjust(TAGS, OFFSET) adjusts the byte range of each
            %   tag in the TAGS list by adding the specified OFFSET value.
            %
            %   TAGS is a list of tags where each tag is a structure with
            %   the fields.
            %       Name      - Name of the tag.
            %       Value     - Data associated with the tag.
            %       ByteRange - Range of bytes within the packet to which the
            %                   tag is applicable.
            %
            %   OFFSET is the number of bytes by which the byte range of
            %   each tag should be shifted. A positive OFFSET shifts the
            %   range forward, while a negative OFFSET shifts it backward.

            % Validate the input arguments
            arguments
                tags (1, :)
                offset (1,1)
            end

            % If the offset is zero, no adjustment is needed; return the
            % original tags
            if offset == 0
                return;
            end

            % Iterate through each tag and adjust the byte range by the
            % given offset
            for idx = 1:numel(tags)
                tags(idx).ByteRange = tags(idx).ByteRange + offset;
            end
        end

        function relevantTags = segment(tags, segmentRange)
            %segment Extract and adjust tags for a specific packet segment
            %
            %   RELEVANTTAGS = segment(TAGS, SEGMENTRANGE) extracts tags that
            %   fall within a specified range of bytes and adjusts their
            %   ByteRange field to be relative to the start of the segment.
            %
            %   TAGS is a list of tags relevant to the original packet. Each
            %   tag is a structure with the fields.
            %       Name      - Name of the tag.
            %       Value     - Data associated with the tag.
            %       ByteRange - Range of bytes within the packet to which the
            %                   tag is applicable.
            %
            %   SEGMENTRANGE is the byte range of segment within the original
            %   packet. This is a 2-element vector. For example, if the segment
            %   of the packet is from byte 10 to byte 20, SEGMENTRANGE would be
            %   [10, 20].
            %
            %   RELEVANTTAGS is a list of tags applicable to the packet
            %   segment. Tags that do not overlap with the segment are excluded.
            %   The byte range of each included tag is adjusted to reflect the
            %   segment's new starting position (byte 1 of the segment).

            % Validate input arguments
            arguments
                tags (1, :)
                segmentRange (1,2)
            end
            validateattributes(segmentRange,{'numeric'},{'nondecreasing'},'packetTags.segment','segmentRange')

            % Preallocate the relevantTags list with the same size as tags
            relevantTags = tags;
            numRelevantTags = 0; % Counter for the number of relevant tags found

            % Define the start and end bytes of the segment for easier reference
            segmentStart = segmentRange(1);
            segmentEnd = segmentRange(2);

            % Iterate over all tags to find those tags that intersect with the
            % segment
            for idx = 1:numel(tags)
                tagStart = tags(idx).ByteRange(1);
                tagEnd = tags(idx).ByteRange(2);

                % Calculate the overlap between the tag's byte range and the
                % segment
                overlapStart = max(tagStart, segmentStart);
                overlapEnd = min(tagEnd, segmentEnd);

                % If there is an overlap, adjust and include the tag in the
                % relevant tags
                if overlapStart <= overlapEnd
                    numRelevantTags = numRelevantTags + 1;
                    relevantTags(numRelevantTags) = tags(idx);
                    relevantTags(numRelevantTags).ByteRange = [overlapStart overlapEnd] - (segmentStart - 1);
                end
            end

            % Remove the unused preallocated elements
            relevantTags = relevantTags(1:numRelevantTags);
        end

        function aggregatedTags = aggregate(varargin)
            %aggregate Combine tags for packet aggregation
            %
            %   AGGREGATEDTAGS = aggregate(TAGS1, PACKETLENGTH1, TAGS2,
            %   PACKETLENGTH2, ...) compiles an updated list of tags appropriate
            %   for the aggregated packet.
            %
            %   TAGS1, TAGS2, ..., TAGSN represent a list of tags. Each list
            %   corresponds to the tags associated with individual packets being
            %   aggregated. Each tag is a structure with the fields.
            %       Name      - Name of the tag.
            %       Value     - Data associated with the tag.
            %       ByteRange - Range of bytes within the packet to which the
            %                   tag is applicable.
            %
            %   PACKETLENGTH1, PACKETLENGTH2, ..., PACKETLENGTHN represent
            %   packet lengths. Each packet length corresponds to an individual
            %   packet being aggregated.
            %
            %   AGGREGATEDTAGS is the resulting list of tags that have been
            %   updated to reflect their new positions within the aggregated
            %   packet.

            % Validate the input arguments
            arguments(Repeating)
                varargin (1, :)
            end
            coder.internal.errorIf(mod(nargin,2) == 1,'MATLAB:system:numArgsMustBeEven');

            aggregatedTags = []; % Initialize the tags list
            byteOffset = 0; % Initialize byte offset for the aggregation

            for idx = 1:2:nargin % Iterate over pairs of tag lists and packet lengths
                tagList = varargin{idx}; % Get the tags for the current packet
                packetLength = varargin{idx+1}; % Get the packet length for the current packet

                % Adjust the tags by the current byte offset
                tagList = wirelessnetwork.internal.packetTags.adjust(tagList, byteOffset);

                % Append the updated tags for the current packet to the
                % aggregated tags
                aggregatedTags = [aggregatedTags tagList];

                % Update the byte offset for the next packet
                byteOffset = byteOffset + packetLength;
            end
        end

        function mergedTags = merge(tags)
            %MERGE Merge tags with same name into a single tag
            %
            %   MERGEDTAGS = MERGE(TAGS) merges tags with same name into a
            %   single tag, adjusting their byte ranges to cover the combined
            %   range of all tags with that name.
            %
            %   MERGEDTAGS is the list of merged tags with updated byte ranges.
            %   Each tag is a structure with the fields.
            %       Name      - Name of the tag.
            %       Value     - Data associated with the tag.
            %       ByteRange - Range of bytes within the packet to which the
            %                   tag is applicable.
            %
            %   TAGS is a list of tags.

            % Validate input arguments
            arguments
                tags (1, :)
            end

            % Preallocate the mergedTags list with the same size as tags
            mergedTags = tags;
            matchCount = 0; % Counter for the number of unique tags found after merging

            % Initialize a map to keep track of unique tag names and their
            % indices in mergedTags
            tagNameMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
            % Iterate through each tag and merge the tags with same
            for idx = 1:numel(tags)
                tag = tags(idx);

                if tagNameMap.isKey(tag.Name)
                    % If the tag name already exists in the map, update the byte
                    % range of the existing tag in mergedTags to include the new
                    % tag's range
                    existingIndex = tagNameMap(tag.Name);
                    mergedTags(existingIndex).ByteRange(1) = min(mergedTags(existingIndex).ByteRange(1), tag.ByteRange(1));
                    mergedTags(existingIndex).ByteRange(2) = max(mergedTags(existingIndex).ByteRange(2), tag.ByteRange(2));
                else
                    matchCount = matchCount+1;
                    % For a new tag name, add it to mergedTags and update the
                    % map with its index
                    tagNameMap(tag.Name) = matchCount;
                    mergedTags(matchCount) = tag;
                end
            end

            % Remove the unused preallocated elements
            mergedTags = mergedTags(1:matchCount);
        end
    end
end