#!/usr/bin/env ruby
# frozen_string_literal: true

require "date"
require "pathname"
require "stringio"
require "yaml"

ROOT = Pathname(__dir__).join("..").realpath
PROJECTS_DIR = ROOT.join("_projects")
ALLOWED_FILTERS = %w[robotics ai control environmental mechanical].freeze

def blank?(value)
  return true if value.nil?
  return value.strip.empty? if value.is_a?(String)
  return value.empty? if value.respond_to?(:empty?)

  false
end

def present?(value)
  !blank?(value)
end

def load_yaml(contents, context)
  YAML.safe_load(contents, permitted_classes: [Date, Time], aliases: true)
rescue Psych::Exception => e
  raise "#{context}: invalid YAML (#{e.message})"
end

def load_front_matter(path)
  contents = path.read
  match = contents.match(/\A---\s*\n(.*?)\n---\s*\n/m)
  raise "#{path.basename}: missing YAML front matter" unless match

  load_yaml(StringIO.new(match[1]), path.basename.to_s) || {}
end

def normalized_array(value)
  Array(value).flatten.compact.reject { |item| blank?(item) }
end

def add_error(errors, message)
  errors << message
end

def validate_collection(errors, project_file, label, items, required_keys)
  unless items.is_a?(Array)
    add_error(errors, "#{project_file}: #{label} must be an array")
    return
  end

  if items.empty?
    add_error(errors, "#{project_file}: #{label} must not be empty")
    return
  end

  items.each_with_index do |item, index|
    unless item.is_a?(Hash)
      add_error(errors, "#{project_file}: #{label} #{index + 1} must be a mapping")
      next
    end

    required_keys.each do |key|
      add_error(errors, "#{project_file}: #{label} #{index + 1} missing `#{key}`") if blank?(item[key])
    end
  end
end

projects = {}
errors = []

Dir[PROJECTS_DIR.join("*.md")].sort.each do |file|
  path = Pathname(file)
  front_matter = load_front_matter(path)
  permalink = front_matter["permalink"]

  if blank?(permalink)
    add_error(errors, "#{path.basename}: missing `permalink` in front matter")
    next
  end

  if projects.key?(permalink)
    add_error(errors, "#{path.basename}: duplicate permalink `#{permalink}` also used by #{projects[permalink]}")
    next
  end

  projects[permalink] = path.basename.to_s

  filter_categories = normalized_array(front_matter["filter_categories"])
  if filter_categories.empty?
    add_error(errors, "#{path.basename}: missing `filter_categories`")
  end

  unknown_filters = filter_categories - ALLOWED_FILTERS
  unless unknown_filters.empty?
    formatted = unknown_filters.map { |item| "`#{item}`" }.join(", ")
    add_error(errors, "#{path.basename}: unknown filter categories #{formatted}")
  end

  if filter_categories.length != filter_categories.uniq.length
    add_error(errors, "#{path.basename}: duplicate filter categories detected")
  end

  title = front_matter["card_title"] || front_matter["title"]
  add_error(errors, "#{path.basename}: missing title/card_title fallback") if blank?(title)

  description = front_matter["card_description"] ||
                front_matter["share-description"] ||
                front_matter["description"]
  add_error(errors, "#{path.basename}: missing description/card_description fallback") if blank?(description)

  add_error(errors, "#{path.basename}: missing `category_label`") if blank?(front_matter["category_label"])

  validate_collection(errors, path.basename.to_s, "impacts", front_matter["impacts"], %w[value label])
  validate_collection(errors, path.basename.to_s, "tech_tags", front_matter["tech_tags"], %w[label style])

  has_media = present?(front_matter["video_url"]) || present?(front_matter["thumbnail-img"])
  add_error(errors, "#{path.basename}: missing `video_url` or `thumbnail-img`") unless has_media
end

if errors.any?
  warn "Project metadata validation failed:"
  errors.each { |message| warn "- #{message}" }
  exit 1
end

puts "Validated #{projects.length} projects with project-level card metadata."
