classdef FigureSaver
    %FIGURESAVER Save open MATLAB figures and write a markdown report section.
    %
    % Usage (from scripts):
    %   [pngRelPaths, runDir] = FigureSaver.saveAllOpen(projectRoot, "stage2");
    %   FigureSaver.appendToReport(reportPath, "Stage 2", pngRelPaths, metaLines);
    %
    % Figures are saved under: docs/figures/<stageTag>/<timestamp>/
    %
    % Notes:
    % - Uses figure Name for stable naming when present.
    % - Designed to work both in desktop and headless runs.

    methods (Static)
        function [pngRelPaths, runDirAbs] = saveAllOpen(projectRoot, stageTag)
            arguments
                projectRoot (1,1) string
                stageTag (1,1) string
            end

            ts = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
            runDirAbs = fullfile(projectRoot, 'docs', 'figures', char(stageTag), char(ts));
            if ~exist(runDirAbs, 'dir')
                mkdir(runDirAbs);
            end

            figs = findall(0, 'Type', 'figure');
            figs = flipud(figs(:)); % older first for nicer numbering

            pngRelPaths = strings(0, 1);
            used = containers.Map('KeyType', 'char', 'ValueType', 'double');

            for i = 1:numel(figs)
                fig = figs(i);

                base = string(fig.Name);
                if strlength(strtrim(base)) == 0
                    base = "Figure_" + string(fig.Number);
                end
                base = FigureSaver.sanitizeFilename(base);

                key = char(base);
                if isKey(used, key)
                    used(key) = used(key) + 1;
                    base = base + "_" + string(used(key));
                else
                    used(key) = 1;
                end

                pngAbs = fullfile(runDirAbs, char(base + ".png"));
                FigureSaver.exportPng(fig, pngAbs);

                rel = string(erase(pngAbs, projectRoot));
                rel = replace(rel, filesep, '/');
                if startsWith(rel, "/") || startsWith(rel, "\")
                    rel = extractAfter(rel, 1);
                end
                pngRelPaths(end+1, 1) = rel; %#ok<AGROW>
            end
        end

        function pngRelPaths = exportSelected(projectRoot, stageTag, figureNames)
            arguments
                projectRoot (1,1) string
                stageTag (1,1) string
                figureNames (:,1) string
            end

            outDirAbs = fullfile(projectRoot, 'docs', 'figures', 'selected', char(stageTag));
            if ~exist(outDirAbs, 'dir')
                mkdir(outDirAbs);
            end

            figs = findall(0, 'Type', 'figure');
            pngRelPaths = strings(0, 1);

            for i = 1:numel(figureNames)
                wanted = figureNames(i);
                match = [];
                for j = 1:numel(figs)
                    if string(figs(j).Name) == wanted
                        match = figs(j);
                        break;
                    end
                end

                if isempty(match)
                    continue;
                end

                base = FigureSaver.sanitizeFilename(wanted);
                pngAbs = fullfile(outDirAbs, char(base + ".png"));
                FigureSaver.exportPng(match, pngAbs);

                rel = string(erase(pngAbs, projectRoot));
                rel = replace(rel, filesep, '/');
                if startsWith(rel, "/") || startsWith(rel, "\")
                    rel = extractAfter(rel, 1);
                end
                pngRelPaths(end+1, 1) = rel; %#ok<AGROW>
            end
        end

        function appendToReport(reportPathAbs, sectionTitle, pngRelPaths, metaLines)
            arguments
                reportPathAbs (1,1) string
                sectionTitle (1,1) string
                pngRelPaths (:,1) string
                metaLines (:,1) string = strings(0,1)
            end

            reportDir = fileparts(reportPathAbs);
            if ~exist(reportDir, 'dir')
                mkdir(reportDir);
            end

            isNew = ~exist(reportPathAbs, 'file');
            fid = fopen(reportPathAbs, 'a');
            if fid < 0
                error('FigureSaver:ReportOpenFailed', 'Failed to open report file: %s', reportPathAbs);
            end
            c = onCleanup(@() fclose(fid));

            if isNew
                fprintf(fid, '# Stage 2 + Stage 3 plots\\n\\n');
                fprintf(fid, 'This file is generated/updated by `scripts/run_stage2_radar_generation.m` and `scripts/run_stage3_tracking.m`.\\n\\n');
            end

            fprintf(fid, '## %s\\n\\n', sectionTitle);
            fprintf(fid, '**Run timestamp**: %s\\n\\n', char(string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'))));

            if ~isempty(metaLines)
                fprintf(fid, '### Metadata\\n\\n');
                for i = 1:numel(metaLines)
                    fprintf(fid, '- %s\\n', char(metaLines(i)));
                end
                fprintf(fid, '\\n');
            end

            fprintf(fid, '### Figures\\n\\n');
            for i = 1:numel(pngRelPaths)
                p = pngRelPaths(i);
                fprintf(fid, '![%s](%s)\\n\\n', char(FigureSaver.escapeAltText(p)), char(p));
            end
            fprintf(fid, '\\n');
        end
    end

    methods (Static, Access = private)
        function exportPng(fig, outPathAbs)
            outDir = fileparts(outPathAbs);
            if ~exist(outDir, 'dir')
                mkdir(outDir);
            end

            try
                if exist('exportgraphics', 'file') == 2
                    exportgraphics(fig, outPathAbs, 'Resolution', 200);
                else
                    set(fig, 'PaperPositionMode', 'auto');
                    print(fig, outPathAbs, '-dpng', '-r200');
                end
            catch me
                warning('FigureSaver:ExportFailed', 'Failed to save %s (%s)', outPathAbs, me.message);
            end
        end

        function s = sanitizeFilename(s)
            s = string(s);
            s = regexprep(s, '\\s+', '_');
            s = regexprep(s, '[\\\\/:*?\"<>|]+', '');
            s = regexprep(s, '[^A-Za-z0-9_\\-\\.]', '');
            s = regexprep(s, '_+', '_');
            s = strip(s, '_');
            if strlength(s) == 0
                s = "Figure";
            end
        end

        function s = escapeAltText(p)
            s = string(p);
            s = replace(s, '[', '\\[');
            s = replace(s, ']', '\\]');
        end
    end
end

